# Sistema integrado Pololu 3Pi+ con TinyS3 (ESP32-S3)

Este programa implementa un sistema embebido completo sobre el **TinyS3 (ESP32-S3)** para controlar el **agente robótico Pololu 3Pi+**, manejar **servomotores** a través de **MCPWM (Prelude)** y mantener un **streaming activo** desde una cámara **OpenMV Cam H7** mediante **SPI + Wi-Fi TCP**.

El sistema está diseñado bajo **ESP-IDF (PlatformIO – espidf)** y ejecuta todas las operaciones de forma concurrente mediante **FreeRTOS**.

---

## Funcionalidad general

El flujo del programa ejecuta una **secuencia automática** de acciones en paralelo con el streaming continuo:

| Etapa | Acción | Duración | Descripción |
|-------|---------|-----------|--------------|
| 1 | Movimiento de ruedas | 10 s | Envía velocidades por UART (CBOR) al Pololu |
|   | Pausa | 2 s | Detiene movimiento |
| 2 | Servos 1 y 2 | 20 s | Oscilan suavemente entre 0° y 90° |
|   | Pausa | 2 s | Breve pausa entre etapas |
| 3 | Servo 3 | 20 s | Movimiento suave entre 0° y 90° |
| — | Streaming | Permanente | Recepción SPI y transmisión TCP de frames 80×60 |

El **streaming** permanece activo durante toda la ejecución, independiente del flujo de control.


