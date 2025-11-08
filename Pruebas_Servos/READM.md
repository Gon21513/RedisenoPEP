# Control de tres servomotores con TinyS3 (ESP32-S3)

Este programa controla tres servomotores utilizando los módulos **MCPWM** del ESP32-S3.  
Además, genera un pulso de encendido inicial hacia el agente **Pololu 3Pi+** mediante un pin de control dedicado.

---

## Descripción general

El sistema configura tres canales PWM independientes para controlar servos en los GPIO 1, 2 y 3, generando pulsos de entre **500 µs y 2400 µs** a una frecuencia de **50 Hz**.  
Los tres servos se mueven simultáneamente entre 0° y 180° con una pausa de 1 segundo en cada extremo.

---

## Funciones principales

### `angle_to_pulsewidth(uint32_t angle)`
Convierte un ángulo entre 0° y 180° en un ancho de pulso en microsegundos dentro del rango de 500 a 2400 µs.

### `setup_servo(mcpwm_unit_t unit, mcpwm_timer_t timer, gpio_num_t pin)`
Configura un canal PWM del ESP32-S3 para controlar un servo:
- Frecuencia de 50 Hz.  
- Modo de conteo ascendente.  
- Señal en canal A (MCPWMxA).  

### `app_main()`
Secuencia principal del programa:
1. Genera un pulso de 100 ms en el pin de control para encender el Pololu.  
2. Inicializa tres módulos MCPWM independientes para los servos.  
3. Alterna las posiciones de los servos entre 0° y 180° cada segundo.  

---

## Configuración de pines

| Señal / Componente | GPIO TinyS3 | Descripción |
|--------------------|-------------|--------------|
| `SERVO1_GPIO` | 1 | Servo 1 controlado por `MCPWM_UNIT_0`, `TIMER_0` |
| `SERVO2_GPIO` | 2 | Servo 2 controlado por `MCPWM_UNIT_0`, `TIMER_1` |
| `SERVO3_GPIO` | 3 | Servo 3 controlado por `MCPWM_UNIT_1`, `TIMER_0` |
| `CTRL_PIN` | 4 | Pulso de encendido hacia el Pololu 3Pi+ |

---

## Parámetros configurables

| Parámetro | Descripción | Valor por defecto |
|------------|-------------|------------------|
| `SERVO_MIN_PULSEWIDTH_US` | Pulso mínimo (0°) | 500 µs |
| `SERVO_MAX_PULSEWIDTH_US` | Pulso máximo (180°) | 2400 µs |
| `SERVO_PERIOD_US` | Periodo del PWM | 20000 µs (50 Hz) |

---

## Comportamiento esperado

1. El pin de control (`CTRL_PIN`) se pone en alto por 100 ms para encender el Pololu.  
2. Los tres servos se mueven a 180° y luego a 0° de forma sincronizada, con una pausa de 1 s entre movimientos.  
3. El ciclo se repite indefinidamente.  

---

## Dependencias

- **ESP-IDF (v5.0 o superior)**  
- Librerías utilizadas:
  - `driver/mcpwm.h`
  - `driver/gpio.h`
  - `freertos/FreeRTOS.h`
  - `freertos/task.h`

---
