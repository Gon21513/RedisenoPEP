# Envío periódico de velocidades al Pololu 3Pi+ mediante TinyS3 (ESP32-S3)

Este programa configura el **TinyS3 (ESP32-S3)** para enviar continuamente las velocidades de las ruedas del agente **Pololu 3Pi+** utilizando comunicación **UART** y codificación **CBOR**.  
El sistema está implementado sobre **FreeRTOS**, donde una tarea dedicada envía los datos cada **100 ms**.


## Descripción general

El TinyS3 actúa como controlador de alto nivel para el Pololu 3Pi+, transmitiendo comandos de velocidad angular (en rpm) de las ruedas izquierda y derecha.  
Las velocidades se codifican en formato **CBOR (Concise Binary Object Representation)** para una comunicación eficiente y estructurada.

El código puede servir como base para integración con módulos de control más complejos (por ejemplo, control PID o interfaces de sensores).


