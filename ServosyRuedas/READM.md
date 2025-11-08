# TinyS3 (ESP32-S3) – Control concurrente Pololu 3Pi+ (UART CBOR) y Servos MCPWM

Este programa implementa un sistema concurrente en **TinyS3 (ESP32-S3)** para:
1. Enviar comandos de velocidad al **Pololu 3Pi+** usando **UART** con codificación **CBOR**.  
2. Controlar tres **servomotores** mediante el módulo **MCPWM** del ESP32-S3.  

El código utiliza **FreeRTOS** para ejecutar ambas tareas en paralelo, manteniendo tiempos precisos y movimiento fluido.

---

## Descripción general

El sistema opera con dos tareas principales:

| Tarea | Núcleo | Descripción |
|--------|--------|-------------|
| `encode_send_wheel_speeds_task` | APP_CPU | Envía velocidades por UART al Pololu cada 100 ms, codificadas con TinyCBOR. |
| `servos_task` | PRO_CPU | Controla tres servos conectados a los GPIO 1, 2 y 3, realizando un barrido 0° ↔ 180°. |

Además, se incluye control de encendido del Pololu mediante un pin digital dedicado.

