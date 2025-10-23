#pragma once

#include "driver/rmt.h"

void ws2812_init(gpio_num_t gpio, uint16_t led_count);
void ws2812_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_refresh(void);
