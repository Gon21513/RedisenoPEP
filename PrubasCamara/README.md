# Minimal Stream Bridge TinyS3 ↔ OpenMV

Este programa implementa un puente de transmisión mínima entre la cámara **OpenMV Cam H7** y el microcontrolador **TinyS3 (ESP32-S3)**.  
Recibe imágenes en tiempo real mediante **SPI** (TinyS3 como esclavo) y las retransmite por **TCP/IP** a través de un punto de acceso Wi-Fi (SoftAP).  


## Descripción general

El sistema permite que otro dispositivo (por ejemplo, una computadora o un cliente Python) se conecte vía Wi-Fi al TinyS3 y solicite los frames capturados por la OpenMV usando un protocolo simple basado en comandos TCP.

Cada frame de 80×60 píxeles se recibe por SPI y se almacena en `rx_buffer`, listo para ser transmitido cuando el cliente envía el comando `'A'`.




