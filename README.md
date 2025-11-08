# Desarrollo de una placa de expansión para los agentes Pololu 3Pi+ que expanda sus capacidades dentro del ecosistema Robotat.

Este trabajo se desarrolló con el objetivo de ampliar las capacidades de los agentes 3Pi+ utilizados en la plataforma de pruebas Robotat. Para aumentar las capacidades del agente, sse implementaron varios servomotores feetech ft90m. También, se implementó una cámara OpenMV Cam H7 que permite capturar desde el punto del agente un fotograma o una serie de fotogramas, para realizar un livestream del agente mientras realiza otras rutinas o capturar una imagen en un punto clave de la ejecución de una rutina. Por último, se implementó un algoritmo para el estacionamiento automático del agente, el cual permite el retorno del agente a una posición establecida mientras evita los obstáculos en la plataforma, lo que permite automatizar el proceso de pruebas y asegurar un correcto funcionamiento del agente. Estas expansiones y el agente son controlados desde un TinyS3, el cual dota al agente y sus módulos con la capacidad de ser controlado por medio de una conexión WiFi, desde un cliente remoto con la capacidad de expandirse a múltiples lenguajes al utilizar instrucciones sencillas para su control.

<center><img src="https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/TitlePic-PhotoRoom.png-PhotoRoom.png?raw=true" width="600" height="600"/></center>

Que contiene este repositorio:
- El firmware utilizado por la placa TinyS3.
- Los códigos para el control del agente y módulos desde Python.
- Manual para el ensamble de las placas de expansion y los modulos.
- Videos de las pruebas para el correcto ensamblaje del agente.
- Enlaces a las pruebas y validación del agente.

### Tabla de contenido:
- [Plataforma de trabajo](#Plataforma-de-trabajo)
- [Estructura del repositorio](#Estructura-del-repositorio)
- [Como ensamblar los modulos y placas](#Como-ensamblar-los-modulos-y-placas)
- [Resultados](#Resultados)

### Plataforma de trabajo

![Platformio y ESPRESSIF](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/piolabs-espressif-partnership.png?raw=true)

El firmware del agente se desarrolló utilizando la plataforma PlatformIO en VSCode, junto con el framework de ESP-IDF desarrollado como una extensión por ESPRESSIF. La elección de VSCode para este proyecto se debe a su alta flexibilidad para configurar el entorno de desarrollo, desde las alertas visuales hasta el manejo de errores y las diversas extensiones para interactuar con módulos y bibliotecas. Además, su fácil integración con GitHub y la gestión de versiones permitieron un desarrollo más eficiente.

Se utilizó la extensión de PlatformIO debido a las diversas herramientas que contiene para programar y preparar los entornos de trabajo para diversas placas y microcontroladores. Esta extensión permite crear proyectos específicos para cada placa, configurar sus características de manera sencilla, seleccionar las versiones del lenguaje a utilizar y manejar los errores de compilación. En el lado físico, permite programar las placas desde VSCode, utilizar el monitor serial para leer los mensajes de error y configurar acorde al puerto y velocidad de comunicación con la placa. Además, facilita compilar el proyecto y verificar los errores al cargar datos a las placas.

El framework de ESP-IDF se eligió principalmente por la fácil integración de FreeRTOS, lo que permite aprovechar al máximo las capacidades de la placa y optimizar su rendimiento multitarea. Además, este framework permite utilizar las bibliotecas desarrolladas por ESPRESSIF para un manejo eficiente de los módulos de las placas basadas en ESP32, proporcionando mayor flexibilidad en su configuración y permitiendo el uso del multiplexado de los pines para los distintos módulos.

![Altium Designer](https://raw.githubusercontent.com/JoseLuisA-P/Tesis-ESP32-Pololu/main/assets/18189621-3f7a-435f-8a02-462efb2cec41.avif)

Para el desarrollo de las PCBs, se utilizó Altium Designer 2022. Aunque este software permite el uso de librerías de objetos comunes, fue necesario agregar librerías personalizadas para cada uno de los componentes adquiridos localmente sin un modelo definido. La elección de Altium Designer se debe a la variedad de herramientas que ofrece para el desarrollo de PCBs y la facilidad para modificar e interconectar proyectos.

![Inventor](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/autodesk-inventor-professiona%C3%B6-1280x720.jpg?raw=true)


# Estructura del repositorio

El repositorio se encuentra organizado en las siguientes carpetas principales:

- **[PruebasCOMS](/PruebasCOMS)**  
  Contiene el proyecto utilizado para la configuración y generación del firmware del **TinyS3 (ESP32-S3)**, incluyendo los módulos de comunicación, control y manejo de periféricos.

- **[assets](/assets)**  
  Incluye las imágenes de referencia utilizadas dentro de este repositorio y en la documentación del proyecto.

- **[Videos](/Videos)**  
  Almacena videos que documentan las pruebas de ensamblaje, integración de componentes y funcionamiento general del sistema.

- **[Manufactura](/Manufactura)**  
  Contiene los archivos necesarios para la **fabricación de las PCBs** y la **impresión 3D** de las piezas diseñadas para este proyecto.

- **[ControlCliente](/ControlCliente)**  
  Incluye los scripts desarrollados en **Python** para el control remoto del agente robótico, la comunicación con el TinyS3 y la visualización de datos.

- **[Camara](/Camara)**  
  Reúne los programas relacionados con la **comunicación y transmisión de video** entre el **TinyS3** y la **OpenMV Cam H7**.  
  Contiene los siguientes subapartados:

  - **openmv_stream**: código que debe cargarse directamente en la cámara OpenMV. Captura imágenes y las envía por SPI hacia el TinyS3.  
  - **PruebaCamara**: script para PC (VS Code, Thonny, etc.) que recibe y visualiza el streaming enviado por el microcontrolador.  
  - **MovimientoCamaraServo**: prueba para validar el control del servomotor encargado de orientar la cámara mediante el módulo MCPWM del TinyS3.  
  - **PruebasCamara**: código general de verificación de la cámara (inicialización, SPI, transmisión). Permite probar el streaming del sistema completo.  
  - **PruebaServoCamara**: ensayo que combina el control del servo y la captura de video, verificando la sincronización entre ambos.  

- **[Pruebas_Servos](/Pruebas_Servos)**  
  Códigos empleados para probar individualmente los servomotores conectados a la placa, comprobando amplitud angular, suavidad de movimiento y calibración PWM.

- **[PruebasBateria](/PruebasBateria)**  
  Programa de medición del nivel de batería mediante el ADC interno del ESP32-S3. Permite validar la correspondencia entre el voltaje medido y el estado real de la celda.

- **[ServosyRuedas](/ServosyRuedas)**  
  Código de prueba que controla únicamente las **ruedas y servos**, sin incluir el módulo de cámara. Se usa para calibración rápida del sistema de tracción.

- **[Servos_Ruedas_CAMARA](/Servos_Ruedas_CAMARA)**  
  Firmware que integra el **control de ruedas, servomotores y cámara** en una secuencia de pruebas completas:
  1. Movimiento de ruedas por un intervalo definido.  
  2. Movimiento de los servos de orientación.  
  3. Activación y verificación del streaming de la cámara.


### Como ensamblar los modulos y placas

Para ensamblar los módulos y placas utilizados en este trabajo, puede hacer referencia al documento **"ManualPololu.pdf"**. En este documento, encontrará una breve descripción de las placas, así como instrucciones detalladas sobre cómo ensamblar las placas y módulos. Además, se proporciona información sobre cómo montarlos y detalles sobre otros materiales adicionales que podrían ser necesarios en el proceso.

En el proceso de ensamblaje, se incluyen referencias a pruebas creadas para observar el correcto funcionamiento del agente antes de desplegarlo, lo que facilita el diagnóstico durante su ensamblaje.

