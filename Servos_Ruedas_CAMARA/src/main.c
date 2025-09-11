// ================================================================
// ESP-IDF (PlatformIO - espidf) - TinyS3 ESP32-S3
// Pololu 3Pi+ por UART (CBOR) + Servos MCPWM  -> al final: Stream OpenMV
// Secuencia: 10s ruedas -> stop -> 2s -> 20s servos 1&2 (5x 0..280..0)
// -> centro -> 2s -> 20s servo 3 (5x 0..180..0) -> centro -> 5s
// => inicia WiFi AP + SPI Slave + TCP server para streaming
// ================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// ======= WiFi / NVS / NETIF / TCP/IP =======
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

// ======= SPI Slave =======
#include "driver/spi_slave.h"

// ======= TinyCBOR (PlatformIO lib_deps = sgoudsme/tinycbor@^0.6.2)
#include "cbor.h"

// ==================== DEFINES Y PINES ====================

// Switch Pololu (ajusta a tu PCB)
static const gpio_num_t PIN_ROBOT_SWITCH = (gpio_num_t)4;  // HIGH = enciende Pololu

// UART2 TinyS3 <-> Pololu
#define UART_PORT        UART_NUM_2
#define UART_BAUD        115200
#define TX_TO_ROBOT      7
#define RX_TO_ROBOT      8
#define UART_TXBUF_SIZE  256
#define UART_RXBUF_SIZE  256

// Servos (MCPWM)
#define SERVO1_GPIO  ((gpio_num_t)1)
#define SERVO2_GPIO  ((gpio_num_t)2)
#define SERVO3_GPIO  ((gpio_num_t)3)
#define SERVO_FREQ_HZ    50
#define SERVO_MIN_US     1000
#define SERVO_MAX_US     2000
#define SERVO_CENTER_US  1500

// Control ruedas
static const unsigned CONTROL_TIME_MS = 100;
static uint8_t uart_send_buffer[32] = {0};

// ======= Streaming (WiFi AP + SPI + TCP) =======
static const char *TAG = "MINI";
#define SSID           "Pololu3Pi+"
#define PASSWORD       "MT30062023"
#define CHANNEL        2
#define MAX_CONN       1
#define PORT           3333

// SPI (TinyS3)
#define SPI_HOST_USED  SPI2_HOST
#define PIN_MISO       GPIO_NUM_37
#define PIN_MOSI       GPIO_NUM_35
#define PIN_SCLK       GPIO_NUM_36
#define PIN_CS         GPIO_NUM_34

// Frame 80x60 (GRAYSCALE)
#define WIDTH          80
#define HEIGHT         60
#define FRAME_BYTES    (WIDTH * HEIGHT)
static uint8_t rx_buffer[FRAME_BYTES];

// ==================== UTILIDADES ====================
static inline uint32_t clamp_u32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return (uint32_t)lo;
    if (v > hi) return (uint32_t)hi;
    return (uint32_t)v;
}

static inline uint32_t deg0_280_to_us(int32_t deg) {
    deg = (int32_t)clamp_u32(deg, 0, 280);
    return SERVO_MIN_US + (uint32_t)((deg * (SERVO_MAX_US - SERVO_MIN_US)) / 280.0f);
}
static inline uint32_t deg0_180_to_us(int32_t deg) {
    deg = (int32_t)clamp_u32(deg, 0, 180);
    return SERVO_MIN_US + (uint32_t)((deg * (SERVO_MAX_US - SERVO_MIN_US)) / 180.0f);
}
static inline float ease_sine_io(float x) {   // 0..1 -> 0..1 suave
    return 0.5f - 0.5f * cosf((float)M_PI * x);
}

// ==================== UART / CBOR (ruedas) ====================
static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    uart_driver_install(UART_PORT, UART_RXBUF_SIZE, UART_TXBUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, TX_TO_ROBOT, RX_TO_ROBOT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void send_wheel_speeds(float phi_left, float phi_right) {
    CborEncoder enc;
    CborEncoder arr;
    cbor_encoder_init(&enc, uart_send_buffer, sizeof(uart_send_buffer), 0);
    cbor_encoder_create_array(&enc, &arr, 2);
    cbor_encode_float(&arr, phi_left);
    cbor_encode_float(&arr, phi_right);
    cbor_encoder_close_container(&enc, &arr);
    size_t n = cbor_encoder_get_buffer_size(&enc, uart_send_buffer);
    uart_write_bytes(UART_PORT, (const char*)uart_send_buffer, n);
}

// ==================== MCPWM (servos) ====================
static void servo_setup(mcpwm_unit_t unit,
                        mcpwm_timer_t timer,
                        mcpwm_io_signals_t io_sig,
                        gpio_num_t gpio) {
    mcpwm_gpio_init(unit, io_sig, gpio);
    mcpwm_config_t cfg = {
        .frequency    = SERVO_FREQ_HZ,
        .cmpr_a       = 0.0f,
        .cmpr_b       = 0.0f,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode    = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(unit, timer, &cfg);
}
static inline void servo_write_us(mcpwm_unit_t u, mcpwm_timer_t t, uint32_t us) {
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    mcpwm_set_duty_in_us(u, t, MCPWM_OPR_A, us);
}

// ===== Barridos multi-ciclo =====
static void sweep_dual_0_280_cycles(float total_seconds, int cycles) {
    const TickType_t period = pdMS_TO_TICKS(10); // ~100 Hz
    float cycle_seconds = total_seconds / (float)cycles;
    int steps_half = (int)((cycle_seconds * 1000.0f / 2.0f) / 10.0f);
    if (steps_half < 1) steps_half = 1;

    for (int c = 0; c < cycles; ++c) {
        for (int k = 0; k < steps_half; ++k) {              // 0 -> 280
            float a = (steps_half <= 1) ? 1.0f : (float)k / (float)(steps_half - 1);
            float alpha = ease_sine_io(a);
            uint32_t us = deg0_280_to_us((int)(alpha * 280.0f));
            servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, us); // S1
            servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, us); // S2
            vTaskDelay(period);
        }
        for (int k = 0; k < steps_half; ++k) {              // 280 -> 0
            float a = (steps_half <= 1) ? 1.0f : (float)k / (float)(steps_half - 1);
            float alpha = ease_sine_io(a);
            uint32_t us = deg0_280_to_us((int)((1.0f - alpha) * 280.0f));
            servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, us);
            servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, us);
            vTaskDelay(period);
        }
    }
}
// 0..90..0 (manteniendo el nombre original para no tocar llamadas)
static void sweep_single_0_180_cycles(float total_seconds, int cycles) {
    const TickType_t period = pdMS_TO_TICKS(10); // ~100 Hz
    float cycle_seconds = total_seconds / (float)cycles;
    int steps_half = (int)((cycle_seconds * 1000.0f / 2.0f) / 10.0f);
    if (steps_half < 1) steps_half = 1;

    for (int c = 0; c < cycles; ++c) {
        // Ida 0 -> 90
        for (int k = 0; k < steps_half; ++k) {
            float a = (steps_half <= 1) ? 1.0f : (float)k / (float)(steps_half - 1);
            float alpha = ease_sine_io(a);
            uint32_t us = deg0_180_to_us((int)(alpha * 90.0f));
            servo_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, us); // S3
            vTaskDelay(period);
        }
        // Vuelta 90 -> 0
        for (int k = 0; k < steps_half; ++k) {
            float a = (steps_half <= 1) ? 1.0f : (float)k / (float)(steps_half - 1);
            float alpha = ease_sine_io(a);
            uint32_t us = deg0_180_to_us((int)((1.0f - alpha) * 90.0f));
            servo_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, us);
            vTaskDelay(period);
        }
    }
}


// ==================== STREAMING: WiFi + SPI + TCP ====================
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .ssid_len = strlen(SSID),
            .channel = CHANNEL,
            .password = PASSWORD,
            .max_connection = MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = true },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP listo: SSID:%s  pass:%s  ch:%d", SSID, PASSWORD, CHANNEL);
}

static void spi_init_slave(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = FRAME_BYTES,
    };
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 2,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_USED, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI Slave listo (80x60=%d bytes)", FRAME_BYTES);
}

static void spi_receive_task(void *arg) {
    spi_slave_transaction_t t;
    while (1) {
        memset(&t, 0, sizeof(t));
        t.length    = FRAME_BYTES * 8;   // bits
        t.tx_buffer = NULL;              // no enviamos nada
        t.rx_buffer = rx_buffer;         // recibimos aquí
        esp_err_t err = spi_slave_transmit(SPI_HOST_USED, &t, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI err=%d", err);
        }
    }
}

static void tcp_server_task(void *arg) {
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) { ESP_LOGE(TAG, "socket fail"); vTaskDelete(NULL); }

    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind fail, errno=%d", errno);
        close(listen_fd); vTaskDelete(NULL);
    }
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "TCP escuchando en %d", PORT);

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int client = accept(listen_fd, (struct sockaddr *)&src, &slen);
        if (client < 0) { ESP_LOGE(TAG, "accept err=%d", errno); continue; }

        char ip[16]; inet_ntoa_r(src.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "Cliente %s conectado", ip);

        while (1) {
            uint8_t cmd;
            int r = recv(client, &cmd, 1, 0);
            if (r <= 0) { ESP_LOGI(TAG, "Cliente cerró"); break; }

            if (cmd == 'A') { // enviar frame actual
                int to_write = FRAME_BYTES, off = 0;
                while (to_write > 0) {
                    int w = send(client, rx_buffer + off, to_write, 0);
                    if (w <= 0) { ESP_LOGE(TAG, "send err=%d", errno); break; }
                    to_write -= w; off += w;
                }
            }
        }
        close(client);
    }
}

// Lanza todo el stack de streaming
static void start_streaming_stack(void) {
    // NVS es requerido por WiFi
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_softap();
    spi_init_slave();

    xTaskCreatePinnedToCore(spi_receive_task, "spi_receive", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_srv",     4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Streaming iniciado");
}

// ==================== ORQUESTADOR (rutina previa) ====================
static void orchestrator_task(void* pv) {
    (void)pv;

    // 1) Ruedas 10 s
    {
        const TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(10 * 1000);
        TickType_t last = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(CONTROL_TIME_MS);
        const float phi = 100.0f; // ambas avanzan
        while (xTaskGetTickCount() < t_end) {
            vTaskDelayUntil(&last, period);
            send_wheel_speeds(phi, phi);
        }
        send_wheel_speeds(0.0f, 0.0f);
    }

    // Espera 2 s
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 2) Servos 1 & 2: 5 ciclos en 20 s (simultáneo 0..280..0)
    sweep_dual_0_280_cycles(20.0f, 5);
    // Centro
    servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, SERVO_CENTER_US);
    servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, SERVO_CENTER_US);

    // Espera 2 s
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 3) Servo 3: 5 ciclos en 20 s (0..180..0)
    sweep_single_0_180_cycles(20.0f, 5);
    // Centro
    servo_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, SERVO_CENTER_US);

    // ===== Pausa final antes del stream =====
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5 segundos

    // ===== Iniciar streaming WiFi+SPI+TCP =====
    start_streaming_stack();

    vTaskDelete(NULL);
}

// ==================== app_main ====================
void app_main(void) {
    // Switch Pololu ON
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ROBOT_SWITCH),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_ROBOT_SWITCH, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_ROBOT_SWITCH, 1);

    // UART hacia Pololu
    uart_init();

    // Servos (MCPWM)
    servo_setup(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, SERVO1_GPIO); // S1
    servo_setup(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, SERVO2_GPIO); // S2
    servo_setup(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, SERVO3_GPIO); // S3

    // Centro inicial
    servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_0, SERVO_CENTER_US);
    servo_write_us(MCPWM_UNIT_0, MCPWM_TIMER_1, SERVO_CENTER_US);
    servo_write_us(MCPWM_UNIT_1, MCPWM_TIMER_0, SERVO_CENTER_US);

    // Lanzar rutina; al final (y +5 s) inicia streaming
    xTaskCreatePinnedToCore(orchestrator_task, "orchestrator", 6144, NULL, 3, NULL, tskNO_AFFINITY);
}
