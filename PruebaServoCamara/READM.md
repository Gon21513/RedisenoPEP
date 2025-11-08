# Control de servomotor con ESP32-S3 usando MCPWM Prelude

Este programa demuestra el uso del nuevo módulo **MCPWM Prelude** del **ESP-IDF** para controlar un **servomotor estándar** conectado al **GPIO1** del TinyS3 (ESP32-S3).  
El sistema genera una señal PWM de **50 Hz** con ancho de pulso variable entre **500 µs** y **2400 µs**, correspondiente a un rango angular de **0° a 180°**.

---

## Descripción general

El código configura manualmente todos los componentes del subsistema **MCPWM**:
1. **Timer:** define la frecuencia y resolución del PWM.  
2. **Operator:** vincula el temporizador con los generadores de salida.  
3. **Comparator:** determina el punto en el que la señal pasa de HIGH a LOW según el ancho de pulso.  
4. **Generator:** produce la señal PWM en el pin configurado (GPIO1).  

El programa mueve el servo de **0° a 90° y de regreso a 0°** en un ciclo continuo de **3 segundos**, con transición suave dividida en múltiples pasos discretos.

---

## Parámetros principales

| Parámetro | Valor | Descripción |
|------------|--------|-------------|
| `SERVO_GPIO` | 1 | Pin de salida PWM hacia el servo |
| `SERVO_MIN_PULSEWIDTH_US` | 500 µs | Pulso mínimo (0°) |
| `SERVO_MAX_PULSEWIDTH_US` | 2400 µs | Pulso máximo (180°) |
| `SERVO_MAX_DEGREE` | 180° | Rango total de movimiento |
| `resolution_hz` | 1 MHz | Resolución de temporizador (1 tick = 1 µs) |
| `period_ticks` | 20000 | Periodo de 20 ms (50 Hz) |
| `pasos` | 75 | Cantidad de pasos en la transición (suavidad) |

---

## Funcionamiento

1. **Inicialización de MCPWM Prelude**
   - Se crean y configuran los objetos `timer`, `operator`, `comparator` y `generator`.  
   - El generador se configura para poner la señal **HIGH** al inicio de cada ciclo y pasar a **LOW** al alcanzar el valor de comparación.

2. **Control de movimiento**
   - En cada iteración se calcula el ancho de pulso correspondiente al ángulo deseado usando la función:
     ```c
     angle_to_pulsewidth(angle);
     ```
   - Se actualiza el valor del comparador con `mcpwm_comparator_set_compare_value(...)` para generar el nuevo pulso.  
   - Se implementa una transición suave mediante pasos discretos y retardos temporizados (`vTaskDelay`).

3. **Ciclo continuo**
   - El servo sube de 0° a 90° durante 1.5 s.  
   - Luego regresa a 0° durante los siguientes 1.5 s.  
   - El proceso se repite indefinidamente.

---

## Diagrama temporal del PWM

