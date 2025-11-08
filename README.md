# Desarrollo de una placa de expansión para los agentes Pololu 3Pi+ que expanda sus capacidades dentro del ecosistema Robotat.

Este trabajo se desarrolló con el objetivo de **ampliar las capacidades de los agentes Pololu 3Pi+** utilizados en la plataforma de pruebas **Robotat**.  
Para lograrlo, se implementaron distintos módulos de expansión y funciones adicionales que extienden el alcance operativo del agente.

En primer lugar, se integraron **servomotores Feetech FT90M**, los cuales permiten dotar al agente de movimiento adicional y control de orientación en componentes periféricos.  
Asimismo, se incorporó una **cámara OpenMV Cam H7**, que permite **capturar imágenes o transmitir video en tiempo real** desde la perspectiva del agente. Esta cámara puede utilizarse tanto para realizar un *livestream* durante la ejecución de rutinas como para capturar fotogramas en momentos clave de operación.

Adicionalmente, se desarrolló un **algoritmo de estacionamiento automático** que posibilita el retorno del agente a una posición predefinida, evitando obstáculos en la plataforma. Esta funcionalidad contribuye a la automatización del proceso de pruebas y asegura un funcionamiento consistente del agente.

Todas las expansiones y el propio agente son controlados por un **TinyS3 (ESP32-S3)**, que proporciona comunicación **Wi-Fi** y permite la interacción desde un **cliente remoto**.  
El sistema fue diseñado para admitir instrucciones simples y extensibles, facilitando la integración con **múltiples lenguajes de programación**.

Este repositorio se basa en y utiliza como punto de partida parte del trabajo original desarrollado por **José Luis Álvarez Pineda**, disponible en el siguiente repositorio:  
👉 [https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu)

El presente proyecto **extiende, adapta y optimiza** dicho trabajo para incorporar nuevas funcionalidades orientadas al control modular, la comunicación con cámara, el manejo de servomotores y la automatización de rutinas del agente dentro del entorno Robotat.


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

Para el desarrollo de las PCBs, se utilizó Altium Designer 2021. Aunque este software permite el uso de librerías de objetos comunes, fue necesario agregar librerías personalizadas para cada uno de los componentes adquiridos localmente sin un modelo definido. La elección de Altium Designer se debe a la variedad de herramientas que ofrece para el desarrollo de PCBs y la facilidad para modificar e interconectar proyectos.

![Inventor](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/autodesk-inventor-professiona%C3%B6-1280x720.jpg?raw=true)

En cuanto al diseño de elementos físicos, se llevó a cabo en Autodesk Inventor 2024. Al contar con una interfaz robusta e intuitiva para el diseño y aprovechando la licencia profesional proporcionada por la universidad, es una herramienta que he utilizado a lo largo de mi carrera. Esta elección permitió minimizar el tiempo de desarrollo gracias a la familiaridad con la herramienta.

### Estructura del repositorio

El repositorio se encuentra estructurado de la siguiente forma:
- En la carpeta [**PruebasCOMS**](/PruebasCOMS), se encuentra el proyecto utilizado para configurar y crear el firmware del TinyS3.
- En la carpeta [**assets**](/assets) se encuentran las imágenes utilizadas para este repositorio.
- En la carpeta [**Videos**](/Videos) se encuentran videos de las pruebas de ensamblaje de este trabajo.
- En la carpeta [**Manufactura**](/Manufactura) se encuentran los archivos para manufacturar las PCBs e imprimir las piezas de este trabajo.
- En la carpeta [**ControlCliente**](/ControlCliente) se encuentran los scripts para el control del agente desde Python.
- En la carpeta [**Camara**](/Camara) se encuentra el script para el envio bajo demanda de la imagen desde la OpenMV Cam H7.

### Como ensamblar los modulos y placas

Para ensamblar los módulos y placas utilizados en este trabajo, puede hacer referencia al documento **"ManualDeEnsamble.pdf"**. En este documento, encontrará una breve descripción de las placas, así como instrucciones detalladas sobre cómo ensamblar las placas y módulos. Además, se proporciona información sobre cómo montarlos y detalles sobre otros materiales adicionales que podrían ser necesarios en el proceso.

En el proceso de ensamblaje, se incluyen referencias a pruebas creadas para observar el correcto funcionamiento del agente antes de desplegarlo, lo que facilita el diagnóstico durante su ensamblaje.

### Resultados

En este archivo se incluye un listado de enlaces a videos de YouTube sin listar, en estos se puede observar el comportamiento del agente y las diversas pruebas realizadas. Las pruebas realizadas incluyen tanto los modulos de manera individual como la integracion de los multiples modulos operando al mismo tiempo.
