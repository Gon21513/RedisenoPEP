El script presentado en esta carpeta permite configurar el envío bajo demanda de los fotogramas capturados por la cámara OpenMV Cam H7 hacia el microcontrolador TinyS3. La configuración de la cámara (como resolución, formato de imagen o frecuencia de captura) puede modificarse según los requerimientos del proyecto para procesar la información necesaria.

Para su funcionamiento, se deben seguir los siguientes pasos:

Cargar en la cámara el script openmv_stream.py utilizando el entorno oficial OpenMV IDE .
Este es el código que se graba directamente en la cámara y permite capturar las imágenes y enviarlas por el bus SPI al TinyS3.

Desde un ordenador, conectarse a la red Wi-Fi generada por el TinyS3 y ejecutar el archivo PruebaCamara.py en un entorno Python, como Visual Studio Code.
Este script permite recibir y visualizar en tiempo real el streaming de video enviado por la cámara a través del microcontrolador.

Con esta configuración, el sistema puede transmitir imágenes en resolución 80×60 píxeles en escala de grises, optimizando el uso de memoria y ancho de banda.
Si se desea enviar otro tipo de datos por SPI, el script puede modificarse para incluir estructuras personalizadas, siempre que se realicen los ajustes correspondientes en la función AskForPicture() del firmware del TinyS3 para procesar los datos recibidos.
