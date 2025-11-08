// ================================================================
// ESP-IDF (PlatformIO - espidf) - TinyS3 ESP32-S3
// Control de servomotores MCPWM (3 servos, secuencia controlada)
// ================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

// ==================== CONFIGURACIÓN GENERAL ====================
static const char *TAG = "SERVOS_SYS";

#define PIN_ROBOT_SWITCH  (gpio_num_t)4

// Servos
#define SERVO1_GPIO  ((gpio_num_t)1)
#define SERVO2_GPIO  ((gpio_num_t)2)
#define SERVO3_GPIO  ((gpio_num_t)3)
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MAX_DEGREE 180

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

// ==================== ORQUESTADOR ====================
static void orchestrator_task(void *pv) {
    // Servos 1 y 2 (20 s aprox)
    ESP_LOGI(TAG, "Servos 1 y 2 activos");
    for (int i = 0; i < 5; i++) {
        servo_move_cycle(&servo1, 90.0f, 4000.0f);
        servo_move_cycle(&servo2, 90.0f, 4000.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Servo 3 (20 s aprox)
    ESP_LOGI(TAG, "Servo 3 activo");
    for (int i = 0; i < 5; i++) {
        servo_move_cycle(&servo3, 90.0f, 4000.0f);
    }

    ESP_LOGI(TAG, "Secuencia finalizada");
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

    servo_init(&servo1, 0, SERVO1_GPIO);
    servo_init(&servo2, 0, SERVO2_GPIO);
    servo_init(&servo3, 1, SERVO3_GPIO);

    // Lanzar la secuencia
    xTaskCreatePinnedToCore(orchestrator_task, "orchestrator", 6144, NULL, 2, NULL, 1);
}