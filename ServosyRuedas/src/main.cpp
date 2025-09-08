// ================================================================
// TinyS3 (ESP32-S3) - Pololu 3Pi+ UART (CBOR) + Servos MCPWM
// Ejecuta tareas concurrentes: envío de velocidades y control de servos
// ================================================================
#include <Arduino.h>
#include <UMS3.h>
#include <tinycbor.h>

// ---- ESP-IDF (servo/MCPWM + GPIO) ----
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/mcpwm.h"
  #include "driver/gpio.h"
  #include "soc/mcpwm_periph.h"
}

// =================== Placa ===================
UMS3 board;

// =================== Pines (AJUSTA A TU PCB) ===================
// Pololu ON/OFF (MOSFET o EN del 3Pi+)
static const uint8_t PIN_ROBOT_SWITCH = 4;     // HIGH = encendido

// UART al Pololu (agente/bridge)
static const uint8_t TX_TO_ROBOT = 7;          // TinyS3 -> Pololu (TX)
static const uint8_t RX_TO_ROBOT = 8;          // Pololu -> TinyS3 (RX)

// Servos (EJEMPLO: CAMBIA A TUS GPIO VÁLIDOS)
#define SERVO1_GPIO  1   // ¡Cambia estos tres!
#define SERVO2_GPIO  2
#define SERVO3_GPIO  3

// =================== CBOR / Control ===================
uint8_t uart_send_buffer[32] = {0};

// Velocidades (rpm) de prueba
volatile float phi_ell = 100.0f;   // izquierda
volatile float phi_r   = -100.0f;  // derecha

// Período de envío (ms)
static const unsigned int CONTROL_TIME_MS = 100;

// =================== Servo (MCPWM) ===================
#define SERVO_MIN_PULSEWIDTH_US  500
#define SERVO_MAX_PULSEWIDTH_US  2400
#define SERVO_PERIOD_US          20000   // 20 ms -> 50 Hz

static inline uint32_t angle_to_pulsewidth(uint32_t angle_deg) {
  if (angle_deg > 180) angle_deg = 180;
  return SERVO_MIN_PULSEWIDTH_US +
         (angle_deg * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180;
}

// Inicializa un canal MCPWM para un servo (operador A)
static void servo_setup(mcpwm_unit_t unit,
                        mcpwm_timer_t timer,
                        mcpwm_io_signals_t io_sig,
                        gpio_num_t gpio)
{
  // Mapea GPIO al IO signal (MCPWMxA)
  mcpwm_gpio_init(unit, io_sig, gpio);

  // Config MCPWM (orden de campos según ESP-IDF)
  mcpwm_config_t pwm_config = {};
  pwm_config.frequency    = 50;                    // 50 Hz para servos
  pwm_config.cmpr_a       = 0.0f;                  // duty A %
  pwm_config.cmpr_b       = 0.0f;                  // duty B % (no usado)
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;

  mcpwm_init(unit, timer, &pwm_config);
}

// =================== Tarea: envío UART (CBOR) ===================
void encode_send_wheel_speeds_task(void* p_params)
{
  (void)p_params;
  TickType_t last_control_time = xTaskGetTickCount();
  const TickType_t period_ticks = pdMS_TO_TICKS(CONTROL_TIME_MS);

  for (;;)
  {
    vTaskDelayUntil(&last_control_time, period_ticks);

    TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
    TinyCBOR.Encoder.create_array(2);
    TinyCBOR.Encoder.encode_float(phi_ell);
    TinyCBOR.Encoder.encode_float(phi_r);
    TinyCBOR.Encoder.close_container();

    Serial2.write(TinyCBOR.Encoder.get_buffer(),
                  TinyCBOR.Encoder.get_buffer_size());
  }
}

// =================== Tarea: barrido de servos ===================
void servos_task(void* p_params)
{
  (void)p_params;

  // Configurar tres salidas (A) en timers distintos para evitar conflictos
  // UNIT_0, TIMER_0 -> MCPWM0A -> SERVO1
  servo_setup(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, (gpio_num_t)SERVO1_GPIO);
  // UNIT_0, TIMER_1 -> MCPWM1A -> SERVO2
  servo_setup(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, (gpio_num_t)SERVO2_GPIO);
  // UNIT_1, TIMER_0 -> MCPWM0A (de UNIT_1) -> SERVO3
  servo_setup(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, (gpio_num_t)SERVO3_GPIO);

  // Poner posición inicial (90°)
  uint32_t pw = angle_to_pulsewidth(90);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pw);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, pw);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pw);

  // Barrido simple 0° <-> 180°
  bool up = true;
  uint32_t ang = 0;

  for (;;)
  {
    // Actualiza ángulo
    if (up) {
      ang += 10;
      if (ang >= 180) { ang = 180; up = false; }
    } else {
      ang = (ang >= 10) ? (ang - 10) : 0;
      if (ang == 0) up = true;
    }

    uint32_t pul = angle_to_pulsewidth(ang);

    // Escribe a los tres servos
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pul);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, pul);
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pul);

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 pasos por segundo
  }
}

// =================== Setup / Loop ===================
void setup()
{
  board.begin();

  // Pulso/estado para encender Pololu
  pinMode(PIN_ROBOT_SWITCH, OUTPUT);
  digitalWrite(PIN_ROBOT_SWITCH, LOW);
  delay(100);
  digitalWrite(PIN_ROBOT_SWITCH, HIGH);

  // UART hacia Pololu
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_TO_ROBOT, TX_TO_ROBOT);

  // TinyCBOR
  TinyCBOR.init();

  // (Opcional) Config GPIO por IDF para el switch (si quieres dejarlo como salida)
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << PIN_ROBOT_SWITCH);
  io_conf.mode         = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)PIN_ROBOT_SWITCH, 1);

  // Tarea UART (alta prioridad para cumplir periodo)
  xTaskCreatePinnedToCore(
    encode_send_wheel_speeds_task,
    "send_wheel_speeds",
    2048,
    nullptr,
    configMAX_PRIORITIES - 2,
    nullptr,
    APP_CPU_NUM
  );

  // Tarea servos (prioridad media)
  xTaskCreatePinnedToCore(
    servos_task,
    "servos_task",
    4096,
    nullptr,
    1,
    nullptr,
    PRO_CPU_NUM
  );
}

void loop()
{
  // Aquí puedes ajustar phi_ell / phi_r dinámicamente si quieres
  // p.ej., leer comandos por WiFi / UART y cambiarlos en caliente.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
