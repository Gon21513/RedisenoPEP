#include <Arduino.h>
#include <UMS3.h>
#include <tinycbor.h>

// Inicializar placa TinyS3
UMS3 board;

// Pines de control
static const uint8_t pin_robot_switch = 4u;  // Activa el Pololu (HIGH = encendido)

// Comunicación UART con el Pololu
static const uint8_t tx_to_robot = 7u;
static const uint8_t rx_to_robot = 8u;
uint8_t uart_send_buffer[32] = {0};

// Velocidades (constantes para prueba)
volatile float phi_ell = 100;   // izquierda, en rpm
volatile float phi_r   = -100;  // derecha, en rpm

// Periodo de envío (100 ms)
static const unsigned int control_time_ms = 100u;

// === Tarea que envía velocidades por UART usando CBOR ===
void encode_send_wheel_speeds_task(void * p_params)
{
  TickType_t last_control_time;
  const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);

  last_control_time = xTaskGetTickCount();

  while (true)
  {
    vTaskDelayUntil(&last_control_time, control_freq_ticks);

    TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
    TinyCBOR.Encoder.create_array(2);
    TinyCBOR.Encoder.encode_float(phi_ell);
    TinyCBOR.Encoder.encode_float(phi_r);
    TinyCBOR.Encoder.close_container();

    Serial2.write(TinyCBOR.Encoder.get_buffer(), TinyCBOR.Encoder.get_buffer_size());
  }
}

void setup()
{
  board.begin();

  // Encender Pololu (pulso inicial)
  pinMode(pin_robot_switch, OUTPUT);
  digitalWrite(pin_robot_switch, LOW);
  delay(100);
  digitalWrite(pin_robot_switch, HIGH);

  // UART hacia Pololu
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, rx_to_robot, tx_to_robot);
  TinyCBOR.init();

  // Crear tarea que envía velocidades cada 100 ms
  xTaskCreate(
    encode_send_wheel_speeds_task,
    "send_wheel_speeds",
    2048,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL
  );
}

void loop()
{
  // Nada aquí — todo corre en tareas
}

