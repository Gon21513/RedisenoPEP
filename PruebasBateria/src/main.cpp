/*
  ---------------------------------------------------------------------
  Código: Monitor de batería para TinyS3 con salida por UART externa
  Autor: Luis Pedro González

  Descripción:
    Este programa mide el voltaje de una batería Li-ion 18650 conectada 
    directamente al pin VBAT del TinyS3, utilizando el ADC interno 
    con atenuación de 11 dB para ampliar el rango de entrada.

    Se aplica un factor de calibración empírico (VBAT_SCALE = 13.08)
    que fue calculado con una batería real midiendo 3.93 V y lectura 
    ADC cruda de 1230. El voltaje resultante se interpreta en función 
    de los umbrales reales típicos de operación de baterías 18650.

    Además, se detecta si el USB está conectado leyendo el pin VBUS_SENSE
    (GPIO33) y se envía el estado por:
      - Serial USB (si está conectado)
      - UART externa (por GPIO2 TX y GPIO3 RX) todo el tiempo

    Para recibir la salida UART incluso cuando el USB del TinyS3 está
    desconectado, se utiliza un módulo FTDI externo o adaptador USB–Serial.
    Este se conecta así:
      - GPIO2 (TX) → RX del FTDI
      - GPIO3 (RX) → TX del FTDI
      - GND       → GND común con el FTDI

    Clasificación de batería:
      - ≥ 4.15 V: completamente cargada
      - 3.95–4.14 V: casi llena
      - 3.7–3.94 V: carga media
      - 3.4–3.69 V: baja
      - 3.0–3.39 V: crítica
      - < 3.0 V: bajo el límite seguro (debería apagarse o entrar en modo de bajo consumo)

    NOTA:
    - Este mapeo es necesario porque el ADC del ESP32-S3 solo mide 
      hasta ~3.6 V con 11 dB de atenuación, mientras que la batería
      puede estar entre 3.0 y 4.2 V. El divisor resistivo interno del TinyS3
      ya escala automáticamente el voltaje de batería, por lo que no se 
      necesita un divisor externo, y el factor VBAT_SCALE corrige esa proporción.

  ---------------------------------------------------------------------
*/


#include <Arduino.h>

// Pines TinyS3
#define VBAT_ADC_PIN     10  // GPIO10 para lectura de batería
#define VBUS_SENSE_PIN   33  // GPIO33 para detección de USB VBUS

// UART externa por HardwareSerial 1 (usando GPIO2 y GPIO3)
#define UART_TX_PIN      2   // TX hacia adaptador USB-Serial
#define UART_RX_PIN      3   // RX desde adaptador USB-Serial

// Calibración empírica afinada: da 3.930 V cuando RAW = 1230
#define VBAT_SCALE       13.08

HardwareSerial uart1(1);  // UART1 para salida externa

void setup() {
  Serial.begin(115200);  // Salida USB (si está disponible)
  uart1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // UART externa

  analogReadResolution(12);  // 12 bits: 0–4095
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);  // Hasta ~3.6 V en pin ADC

  pinMode(VBUS_SENSE_PIN, INPUT);  // Leer si USB está conectado
}

void loop() {
  // Leer voltaje de batería (promedio de 10 lecturas)
  int raw = 0;
  for (int i = 0; i < 10; i++) {
    raw += analogRead(VBAT_ADC_PIN);
    delay(5);
  }
  raw /= 10;

  float voltage = raw * VBAT_SCALE / 4095.0;
  bool usb_connected = digitalRead(VBUS_SENSE_PIN);

  // Armar mensaje
  String msg = "RAW: " + String(raw) + " | Vbat: " + String(voltage, 3) +
               " V | USB: " + (usb_connected ? "Conectado" : "Desconectado") + "\n";

  if (usb_connected) {
    if (voltage >= 4.15f) {
      msg += "Batería completamente cargada.\n";
    } else if (voltage >= 3.95f) {
      msg += "Batería casi llena.\n";
    } else {
      msg += "USB conectado pero batería aún no está llena.\n";
    }
  } else {
    if (voltage >= 4.15f) {
      msg += "Batería completamente cargada.\n";
    } else if (voltage >= 3.8f) {
      msg += "Batería casi llena.\n";
    } else if (voltage >= 3.7f) {
      msg += "Batería con carga media.\n";
    } else if (voltage >= 3.4f) {
      msg += "Batería baja.\n";
    } else if (voltage >= 3.0f) {
      msg += "Batería crítica.\n";
    } else {
      msg += "Advertencia: voltaje bajo el límite seguro.\n";
    }
  }

  // Salida por USB si está presente
  if (usb_connected) Serial.print(msg);

  // Salida siempre por UART externa
  uart1.print(msg);

  delay(1000);
}



