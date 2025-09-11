#include <Arduino.h>
#include <tinycbor.h>
extern "C" {
  #include "driver/mcpwm.h"
  #include "driver/gpio.h"
  #include "soc/mcpwm_periph.h"
}

// -------------------- Pines (ajusta si es necesario) --------------------
#define PIN_ROBOT_SWITCH   4       // HIGH = enciende Pololu
#define TX_TO_ROBOT        7       // TinyS3 TX -> Pololu RX
#define RX_TO_ROBOT        8       // TinyS3 RX <- Pololu TX

// Servo en GPIO1
#define SERVO_GPIO                 1
#define SERVO_MIN_PULSEWIDTH_US  500
#define SERVO_MAX_PULSEWIDTH_US 2400

// -------------------- Comunicación ruedas (CBOR) --------------------
static uint8_t uart_send_buffer[32]{};
static const uint32_t WHEELS_PERIOD_MS = 100;   // envíos cada 100 ms
float phi_left_rpm  = 100.0f;   // ajusta según tu orientación
float phi_right_rpm = -100.0f;  // ajusta según tu orientación

// Para alternar sentido cada 3 s (opcional)
static const uint32_t TOGGLE_DIR_MS = 3000;
uint32_t last_toggle_ms = 0;
bool forward = true;

// -------------------- Servo helpers --------------------
static inline uint32_t angle_to_pulsewidth(int angle_deg) {
  if (angle_deg < 0)   angle_deg = 0;
  if (angle_deg > 180) angle_deg = 180;
  return SERVO_MIN_PULSEWIDTH_US +
         (uint32_t)((angle_deg * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180);
}

// Inicializa MCPWM para el servo (MCPWM_UNIT_0, TIMER_0, OP A)
void setup_servo(gpio_num_t gpio) {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio);
  mcpwm_config_t cfg = {};
  cfg.frequency = 50;               // 50 Hz típico
  cfg.cmpr_a = 0;                   // duty inicial A
  cfg.cmpr_b = 0;                   // no usamos canal B
  cfg.counter_mode = MCPWM_UP_COUNTER;
  cfg.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);

  // Posición inicial (0 lógico -> 90 físico)
  int phys = 90; // 0 lógico
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_to_pulsewidth(phys));
}

// Mapea ángulo lógico [-90, +90] a físico [0, 180]
static inline int logical_to_physical_deg(int ang_log) {
  // -90 -> 0,   0 -> 90,   +90 -> 180
  return ang_log + 90;
}

// -------------------- Envío CBOR ruedas --------------------
void send_wheel_speeds(float left_rpm, float right_rpm) {
  TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
  TinyCBOR.Encoder.create_array(2);
  TinyCBOR.Encoder.encode_float(left_rpm);
  TinyCBOR.Encoder.encode_float(right_rpm);
  TinyCBOR.Encoder.close_container();
  Serial2.write(TinyCBOR.Encoder.get_buffer(), TinyCBOR.Encoder.get_buffer_size());
}

// -------------------- Setup --------------------
void setup() {
  // Pololu ON
  pinMode(PIN_ROBOT_SWITCH, OUTPUT);
  digitalWrite(PIN_ROBOT_SWITCH, LOW);
  delay(100);
  digitalWrite(PIN_ROBOT_SWITCH, HIGH);

  // Consola y UART hacia Pololu
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_TO_ROBOT, TX_TO_ROBOT);

  // CBOR
  TinyCBOR.init();

  // Servo
  setup_servo((gpio_num_t)SERVO_GPIO);
}

// -------------------- Loop --------------------
void loop() {
  static uint32_t last_wheels_ms = 0;
  static uint32_t last_servo_ms  = 0;

  // --------- 1) Movimiento de ruedas (cada 100 ms) ---------
  uint32_t now = millis();

  // Alternar dirección cada 3 s (opcional)
  if (now - last_toggle_ms >= TOGGLE_DIR_MS) {
    last_toggle_ms = now;
    forward = !forward;
    if (forward) {
      phi_left_rpm  = 100.0f;
      phi_right_rpm = -100.0f;
    } else {
      phi_left_rpm  = -100.0f;
      phi_right_rpm = 100.0f;
    }
  }

  if (now - last_wheels_ms >= WHEELS_PERIOD_MS) {
    last_wheels_ms = now;
    send_wheel_speeds(phi_left_rpm, phi_right_rpm);
  }

  // --------- 2) Oscilación del servo (-90° a +90° y regreso) ---------
  // Movimiento por pasos discretos no bloqueante
  static int ang_log = -90;   // ángulo lógico actual [-90, +90]
  static int step    = 5;     // tamaño de paso lógico
  const uint32_t SERVO_STEP_MS = 200;

  if (now - last_servo_ms >= SERVO_STEP_MS) {
    last_servo_ms = now;

    // Actualiza ángulo lógico
    ang_log += step;
    if (ang_log >= +90) { ang_log = +90; step = -step; }
    if (ang_log <= -90) { ang_log = -90; step = -step; }

    int phys = logical_to_physical_deg(ang_log);                // [0..180]
    uint32_t pw = angle_to_pulsewidth(phys);                    // pulso us
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pw);
  }

  // Cede CPU
  delay(1);
}
