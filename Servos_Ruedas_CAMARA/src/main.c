// ================================================================
// ESP-IDF (PlatformIO - espidf) - TinyS3 ESP32-S3
// Pololu 3Pi+ por UART (CBOR) + Servos MCPWM (Prelude) + Streaming activo
// Secuencia controlada: ruedas 10s -> pausa -> servos 1&2 (20s) -> pausa -> servo 3 (20s)
// Streaming permanece disponible en todo momento
// ================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "driver/spi_slave.h"
#include "driver/mcpwm_prelude.h"
#include "cbor.h"

// ==================== CONFIGURACIÓN GENERAL ====================
static const char *TAG = "POLULU_SYS";

#define PIN_ROBOT_SWITCH  (gpio_num_t)4

// UART TinyS3 ↔ Pololu
#define UART_PORT        UART_NUM_2
#define UART_BAUD        115200
#define TX_TO_ROBOT      7
#define RX_TO_ROBOT      8
#define UART_TXBUF_SIZE  256
#define UART_RXBUF_SIZE  256

// Servos
#define SERVO1_GPIO  ((gpio_num_t)1)
#define SERVO2_GPIO  ((gpio_num_t)2)
#define SERVO3_GPIO  ((gpio_num_t)3)
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MAX_DEGREE 180

// WiFi + Streaming
#define SSID       "Pololu3Pi+"
#define PASSWORD   "MT30062023"
#define PORT       3333
#define FRAME_BYTES 4800
#define WIDTH  80
#define HEIGHT 60

// SPI
#define SPI_HOST_USED  SPI2_HOST
#define PIN_MISO       GPIO_NUM_37
#define PIN_MOSI       GPIO_NUM_35
#define PIN_SCLK       GPIO_NUM_36
#define PIN_CS         GPIO_NUM_34

static uint8_t rx_buffer[FRAME_BYTES];

// ==================== UART / CBOR ====================
static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT, UART_RXBUF_SIZE, UART_TXBUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, TX_TO_ROBOT, RX_TO_ROBOT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void send_wheel_speeds(float left, float right) {
    uint8_t buf[32];
    CborEncoder enc, arr;
    cbor_encoder_init(&enc, buf, sizeof(buf), 0);
    cbor_encoder_create_array(&enc, &arr, 2);
    cbor_encode_float(&arr, left);
    cbor_encode_float(&arr, right);
    cbor_encoder_close_container(&enc, &arr);
    size_t n = cbor_encoder_get_buffer_size(&enc, buf);
    uart_write_bytes(UART_PORT, (const char*)buf, n);
}

// ==================== MCPWM SERVOS ====================
typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmp;
    mcpwm_gen_handle_t gen;
} servo_t;

static servo_t servo1, servo2, servo3;

static inline uint32_t angle_to_pulse(float angle) {
    return SERVO_MIN_PULSEWIDTH_US + (uint32_t)((angle / SERVO_MAX_DEGREE) *
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US));
}

static void servo_init(servo_t *s, int group, gpio_num_t pin) {
    mcpwm_timer_config_t tc = {
        .group_id = group,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1e6,
        .period_ticks = 20000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_new_timer(&tc, &s->timer);

    mcpwm_operator_config_t oc = { .group_id = group };
    mcpwm_new_operator(&oc, &s->oper);
    mcpwm_operator_connect_timer(s->oper, s->timer);

    mcpwm_comparator_config_t cc = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(s->oper, &cc, &s->cmp);

    mcpwm_generator_config_t gc = { .gen_gpio_num = pin };
    mcpwm_new_generator(s->oper, &gc, &s->gen);

    mcpwm_generator_set_actions_on_timer_event(
        s->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(
        s->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s->cmp,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    mcpwm_timer_enable(s->timer);
    mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_START_NO_STOP);
}

// Movimiento suave
static void servo_move_cycle(servo_t *s, float max_angle, float total_ms) {
    const int steps = 75;
    float half_ms = total_ms / 2.0f;
    float delay_ms = half_ms / steps;

    for (int i = 0; i <= steps; i++) {
        float angle = (max_angle * i) / steps;
        mcpwm_comparator_set_compare_value(s->cmp, angle_to_pulse(angle));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    for (int i = steps; i >= 0; i--) {
        float angle = (max_angle * i) / steps;
        mcpwm_comparator_set_compare_value(s->cmp, angle_to_pulse(angle));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ==================== STREAMING ====================
static void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wcfg = {
        .ap = {
            .ssid = SSID,
            .ssid_len = strlen(SSID),
            .channel = 2,
            .password = PASSWORD,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
        }
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wcfg);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi AP listo: %s", SSID);
}

static void spi_init_slave(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .max_transfer_sz = FRAME_BYTES
    };
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 2
    };
    spi_slave_initialize(SPI_HOST_USED, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

static void spi_receive_task(void *arg) {
    spi_slave_transaction_t t;
    while (1) {
        memset(&t, 0, sizeof(t));
        t.length = FRAME_BYTES * 8;
        t.rx_buffer = rx_buffer;
        spi_slave_transmit(SPI_HOST_USED, &t, portMAX_DELAY);
    }
}

static void tcp_server_task(void *arg) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(s, (struct sockaddr*)&addr, sizeof(addr));
    listen(s, 1);
    ESP_LOGI(TAG, "TCP escuchando en puerto %d", PORT);

    while (1) {
        struct sockaddr_in src; socklen_t sl = sizeof(src);
        int c = accept(s, (struct sockaddr*)&src, &sl);
        if (c < 0) continue;
        ESP_LOGI(TAG, "Cliente conectado");

        while (1) {
            uint8_t cmd;
            int r = recv(c, &cmd, 1, 0);
            if (r <= 0) break;
            if (cmd == 'A') send(c, rx_buffer, FRAME_BYTES, 0);
        }
        close(c);
        ESP_LOGI(TAG, "Cliente desconectado");
    }
}

static void start_streaming(void) {
    nvs_flash_init();
    wifi_init_softap();
    spi_init_slave();
    xTaskCreatePinnedToCore(spi_receive_task, "spi_rx", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_srv", 4096, NULL, 5, NULL, 0);
}

// ==================== ORQUESTADOR ====================
static void orchestrator_task(void *pv) {
    // 1) Movimiento de ruedas
    ESP_LOGI(TAG, "Ruedas ON (10s)");
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(10000)) {
        send_wheel_speeds(100.0f, 100.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    send_wheel_speeds(0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 2) Servos 1 y 2 (20 s aprox)
    ESP_LOGI(TAG, "Servos 1 y 2 activos");
    for (int i = 0; i < 5; i++) {
        servo_move_cycle(&servo1, 90.0f, 4000.0f);
        servo_move_cycle(&servo2, 90.0f, 4000.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 3) Servo 3 (20 s aprox)
    ESP_LOGI(TAG, "Servo 3 activo");
    for (int i = 0; i < 5; i++) {
        servo_move_cycle(&servo3, 90.0f, 4000.0f);
    }

    ESP_LOGI(TAG, "Secuencia finalizada (streaming sigue activo)");
    vTaskDelete(NULL);
}

// ==================== MAIN ====================
void app_main(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_ROBOT_SWITCH),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
    gpio_set_level(PIN_ROBOT_SWITCH, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_ROBOT_SWITCH, 1);

    uart_init();
    start_streaming();

    servo_init(&servo1, 0, SERVO1_GPIO);
    servo_init(&servo2, 0, SERVO2_GPIO);
    servo_init(&servo3, 1, SERVO3_GPIO);

    // Lanzar la secuencia
    xTaskCreatePinnedToCore(orchestrator_task, "orchestrator", 6144, NULL, 2, NULL, 1);
}
