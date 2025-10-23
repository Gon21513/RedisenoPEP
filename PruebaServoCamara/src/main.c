#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define SERVO_GPIO 1
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MAX_DEGREE 180

static const char *TAG = "SERVO";

// Convierte ángulo (0–180°) a ancho de pulso (µs)
static inline uint32_t angle_to_pulsewidth(float angle)
{
    return SERVO_MIN_PULSEWIDTH_US +
           (uint32_t)((angle / SERVO_MAX_DEGREE) *
           (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando control de servo en GPIO %d...", SERVO_GPIO);

    // === CONFIGURAR TIMER ===
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1e6,      // 1 tick = 1 µs
        .period_ticks = 20000,     // 20 ms → 50 Hz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // === CONFIGURAR OPERADOR ===
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // === CONFIGURAR COMPARATOR ===
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // === CONFIGURAR GENERADOR ===
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = { .gen_gpio_num = SERVO_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // HIGH al inicio del ciclo, LOW al llegar a la comparación
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()
    ));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // === MOVIMIENTO CADA 3 SEGUNDOS (1.5 s ida, 1.5 s vuelta) ===
    const float tiempo_total_ms = 3000.0f;
    const float tiempo_mitad_ms = tiempo_total_ms / 2.0f;
    const int pasos = 75;                    // más pasos = movimiento más suave
    const float delay_ms = tiempo_mitad_ms / pasos;  // tiempo entre pasos

    while (true)
    {
        // Subida 0° → 90°
        for (int i = 0; i <= pasos; i++)
        {
            float angle = (90.0f * i) / pasos;
            uint32_t pulse = angle_to_pulsewidth(angle);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse));
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        // Bajada 90° → 0°
        for (int i = pasos; i >= 0; i--)
        {
            float angle = (90.0f * i) / pasos;
            uint32_t pulse = angle_to_pulsewidth(angle);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse));
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

