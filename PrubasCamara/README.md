# Minimal Stream Bridge TinyS3 ↔ OpenMV

Este programa implementa un **puente de transmisión mínima** entre la cámara **OpenMV Cam H7** y el microcontrolador **TinyS3 (ESP32-S3)**.  
Recibe imágenes en tiempo real mediante **SPI** (TinyS3 como esclavo) y las retransmite por **TCP/IP** a través de un punto de acceso Wi-Fi (SoftAP).  

---

## Descripción general

El sistema permite que otro dispositivo (por ejemplo, una computadora o un cliente Python) se conecte vía Wi-Fi al TinyS3 y solicite los frames capturados por la OpenMV usando un protocolo simple basado en comandos TCP.

Cada frame de **80×60 píxeles** se recibe por SPI y se almacena en `rx_buffer`, listo para ser transmitido cuando el cliente envía el comando `'A'`.

---

## Configuración previa

Antes de ejecutar este programa en el **TinyS3**, deben cargarse los dos scripts necesarios para el funcionamiento del sistema de transmisión que se encuentran en la carpta de  **[Camara]**:

1. **En la cámara OpenMV Cam H7:**  
   Cargar el script **`openmv_stream.py`** utilizando el entorno oficial **OpenMV IDE [23]**.  
   Este código se graba directamente en la cámara y se encarga de capturar los fotogramas y enviarlos por el bus **SPI** hacia el TinyS3.

2. **En el ordenador o cliente remoto:**  
   Conectarse a la red Wi-Fi generada por el TinyS3 e **iniciar el script `PruebaCamara.py`** en un entorno **Python**, como Visual Studio Code.
   Este programa recibe y muestra el *streaming* de video transmitido por la cámara a través del microcontrolador.

---

## Funcionamiento

Una vez configurados ambos scripts:

- La **OpenMV** envía los frames en escala de grises (80×60 píxeles) mediante SPI.  
- El **TinyS3** actúa como puente (*bridge*), recibiendo los datos y retransmitiéndolos por TCP a los clientes conectados a su red Wi-Fi.  
- Un cliente puede solicitar la imagen actual enviando el comando `'A'`, tras lo cual el TinyS3 devuelve el frame más reciente.

---
