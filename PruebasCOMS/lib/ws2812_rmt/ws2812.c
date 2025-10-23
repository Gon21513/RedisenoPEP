#include "ws2812.h"
#include <stdlib.h>
#include <string.h>

#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_HZ 10000000 // 10 MHz
#define T0H 350
#define T0L 800
#define T1H 700
#define T1L 600

static uint8_t *led_data = NULL;
static int led_count = 0;
static gpio_num_t gpio;

void ws2812_init(gpio_num_t pin, uint16_t count)
{
    gpio = pin;
    led_count = count;
    led_data = calloc(led_count * 3, sizeof(uint8_t));

    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = gpio,
        .clk_div = 1,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }};
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void ws2812_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= led_count) return;
    led_data[index * 3 + 0] = g;
    led_data[index * 3 + 1] = r;
    led_data[index * 3 + 2] = b;
}

void ws2812_refresh()
{
    int num_bits = led_count * 24;
    rmt_item32_t *items = calloc(num_bits, sizeof(rmt_item32_t));

    for (int i = 0; i < led_count * 3; i++) {
        uint8_t byte = led_data[i];
        for (int j = 0; j < 8; j++) {
            bool bit = byte & (1 << (7 - j));
            int k = i * 8 + j;
            items[k].level0 = 1;
            items[k].duration0 = bit ? T1H : T0H;
            items[k].level1 = 0;
            items[k].duration1 = bit ? T1L : T0L;
        }
    }

    rmt_write_items(RMT_CHANNEL, items, num_bits, true);
    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
    free(items);
}

