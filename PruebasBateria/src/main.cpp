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



