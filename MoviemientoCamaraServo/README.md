# Control de Servo y Movimiento del Agente Pololu 3Pi+ mediante TinyS3

Este programa configura el microcontrolador **TinyS3 (ESP32-S3)** para controlar simultáneamente:
- El movimiento diferencial del agente robótico **Pololu 3Pi+**, enviando velocidades de las ruedas mediante el protocolo **CBOR** vía UART.
- El movimiento oscilante de un servomotor a través del periférico **MCPWM**, generando pulsos entre 500 µs y 2400 µs a 50 Hz.

---

## Características principales

- Comunicación serial CBOR: envía pares de velocidades (izquierda y derecha) al agente Pololu cada 100 ms.  
- Control de servo no bloqueante: oscila de -90° a +90° de manera continua.  
- Alternancia de dirección automática: invierte el sentido de las ruedas cada 3 s.  
- Inicialización completa de hardware: UART, GPIO, MCPWM y pin de encendido del robot.  

---

## Configuración de pines

| Elemento              | Pin TinyS3 | Descripción                                      |
|------------------------|------------|--------------------------------------------------|
| `PIN_ROBOT_SWITCH`     | 4          | Controla encendido del Pololu (HIGH = ON)        |
| `TX_TO_ROBOT`          | 7          | Transmisión UART hacia el Pololu                 |
| `RX_TO_ROBOT`          | 8          | Recepción UART desde el Pololu                   |
| `SERVO_GPIO`           | 1          | Control PWM del servomotor                       |

---

## Descripción del funcionamiento

1. **Inicialización (`setup`):**  
   - Activa el Pololu.  
   - Configura las interfaces seriales (`Serial`, `Serial2`).  
   - Inicializa la librería TinyCBOR y el módulo MCPWM para el servo.

2. **Bucle principal (`loop`):**
   - Envía periódicamente las velocidades de las ruedas por CBOR (`send_wheel_speeds`).  
   - Alterna el sentido de movimiento cada 3 s (opcional).  
   - Oscila el servo entre -90° y +90° mediante pasos discretos de 5° cada 200 ms.  
   - No bloquea la CPU; mantiene ejecución fluida.

---

## Estructura del mensaje CBOR

El mensaje enviado al Pololu es un array de dos elementos:
```cbor
[ <float:phi_left_rpm>, <float:phi_right_rpm> ]
