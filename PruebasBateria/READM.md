# Monitor de voltaje de batería Li-ion 18650 con TinyS3 (ESP32-S3)

Este programa mide el voltaje de una batería **Li-ion 18650** conectada directamente al pin **VBAT** del **TinyS3**, utilizando el ADC interno con atenuación de **11 dB** para ampliar el rango de entrada.  
También detecta si el **USB** está conectado mediante el pin **VBUS_SENSE (GPIO33)** y envía la información tanto por el puerto **Serial USB** como por una **UART externa** (GPIO2 y GPIO3).

---

## Descripción del funcionamiento

1. **Lectura de voltaje:**
   - Se realiza una lectura promedio de 10 muestras del ADC en el pin VBAT.
   - Se aplica un factor de calibración empírico (`VBAT_SCALE = 13.08`), calculado usando una batería real con:
     - Voltaje real medido: **3.93 V**
     - Lectura ADC cruda: **1230**
   - El valor resultante representa el voltaje real de la batería.

2. **Clasificación de nivel de batería:**

   | Rango de voltaje (V) | Estado |
   |----------------------:|--------|
   | ≥ 4.15 | Completamente cargada |
   | 3.95 – 4.14 | Casi llena |
   | 3.70 – 3.94 | Carga media |
   | 3.40 – 3.69 | Baja |
   | 3.00 – 3.39 | Crítica |
   | < 3.00 | Bajo el límite seguro (debería apagarse o entrar en bajo consumo) |

3. **Detección de USB:**
   - Se lee el pin `VBUS_SENSE (GPIO33)` para determinar si el dispositivo está conectado a alimentación por USB.
   - El estado de conexión se incluye en el mensaje de salida.

4. **Comunicación serial:**
   - **USB (Serial):** solo transmite cuando el TinyS3 está conectado por USB.  
   - **UART externa:** transmite siempre, permitiendo monitoreo con un adaptador **USB–Serial (FTDI)**.

---

## Conexión de la UART externa

| Señal TinyS3 | Adaptador USB–Serial (FTDI) |
|---------------|-----------------------------|
| GPIO2 (TX) | RX |
| GPIO3 (RX) | TX |
| GND | GND común |

Esto permite recibir los datos aun cuando el puerto USB del TinyS3 esté desconectado.

---

## Pines utilizados

| Función | GPIO TinyS3 | Descripción |
|----------|--------------|-------------|
| `VBAT_ADC_PIN` | 10 | Lectura del voltaje de batería |
| `VBUS_SENSE_PIN` | 33 | Detección del estado del USB |
| `UART_TX_PIN` | 2 | Transmisión hacia adaptador USB–Serial |
| `UART_RX_PIN` | 3 | Recepción desde adaptador USB–Serial |

---

## Calibración y consideraciones

- **Atenuación:** `ADC_11db`, permitiendo medir hasta ~3.6 V en el pin ADC.  
- **Factor de escala (`VBAT_SCALE`):** 13.08, corrige el divisor resistivo interno del TinyS3 para mapear correctamente el rango de 3.0–4.2 V de la batería.  
- **No requiere divisor resistivo externo.**  
- El mapeo de voltajes es necesario debido a las limitaciones de entrada del ADC interno del ESP32-S3.



