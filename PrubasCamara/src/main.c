// main.c — Minimal Stream Bridge TinyS3 <-> OpenMV
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

static const char *TAG = "MINI";

// ======= WiFi AP =======
#define SSID           "Pololu3Pi+"
#define PASSWORD       "MT30062023"
#define CHANNEL        2
#define MAX_CONN       1

#define PORT           3333

// ======= SPI pins (TinyS3) =======
#define SPI_HOST_USED  SPI2_HOST
#define PIN_MISO       GPIO_NUM_37
#define PIN_MOSI       GPIO_NUM_35
#define PIN_SCLK       GPIO_NUM_36
#define PIN_CS         GPIO_NUM_34

// ======= Frame =======
#define WIDTH          80
#define HEIGHT         60
#define FRAME_BYTES    (WIDTH * HEIGHT)
static uint8_t rx_buffer[FRAME_BYTES];
static SemaphoreHandle_t frame_mux;

// ======= WiFi =======
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

// ======= SPI esclavo =======
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
        if (err == ESP_OK) {
            // Señal opcional para lector
            // ESP_LOGI(TAG, "Frame SPI OK");
            // Nada más que hacer: rx_buffer ya tiene los datos más recientes
        } else {
            ESP_LOGE(TAG, "SPI err=%d", err);
        }
        // Respira un poco (no obligatorio)
        // vTaskDelay(1);
    }
}

// ======= TCP server =======
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

        // loop simple: lee 1 byte cmd y responde
        while (1) {
            uint8_t cmd;
            int r = recv(client, &cmd, 1, 0);
            if (r <= 0) { ESP_LOGI(TAG, "Cliente cerró"); break; }

            if (cmd == 'A') {
                // enviar frame actual
                int to_write = FRAME_BYTES, off = 0;
                while (to_write > 0) {
                    int w = send(client, rx_buffer + off, to_write, 0);
                    if (w <= 0) { ESP_LOGE(TAG, "send err=%d", errno); break; }
                    to_write -= w; off += w;
                }
            } else {
                // ignora otros comandos en este mini
            }
        }
        close(client);
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_softap();

    spi_init_slave();

    xTaskCreatePinnedToCore(spi_receive_task, "spi_receive", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_srv",    4096, NULL, 5, NULL, 0);
}
