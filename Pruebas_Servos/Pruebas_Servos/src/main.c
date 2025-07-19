#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_PERIOD_US 20000

#define SERVO1_GPIO 1
#define SERVO2_GPIO 2
#define SERVO3_GPIO 3
#define CTRL_PIN     4

uint32_t angle_to_pulsewidth(uint32_t angle) {
    return SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180;
}

void setup_servo(mcpwm_unit_t unit, mcpwm_timer_t timer, gpio_num_t pin) {
    mcpwm_gpio_init(unit, MCPWM0A, pin);
    mcpwm_config_t pwm_config = {
        .frequency = 50, // 50Hz para servos
        .cmpr_a = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(unit, timer, &pwm_config);
}

void app_main() {
    // Pulso de encendido al Pololu
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CTRL_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(CTRL_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_direction(CTRL_PIN, GPIO_MODE_INPUT);

    // Configurar servos
    setup_servo(MCPWM_UNIT_0, MCPWM_TIMER_0, SERVO1_GPIO);
    setup_servo(MCPWM_UNIT_0, MCPWM_TIMER_1, SERVO2_GPIO);
    setup_servo(MCPWM_UNIT_1, MCPWM_TIMER_0, SERVO3_GPIO);

    while (1) {
        uint32_t high_us = angle_to_pulsewidth(180);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, high_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, high_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, high_us);
        vTaskDelay(pdMS_TO_TICKS(1000));

        high_us = angle_to_pulsewidth(0);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, high_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, high_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, high_us);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
