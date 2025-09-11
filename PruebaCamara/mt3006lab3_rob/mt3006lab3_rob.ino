// ================================================================================
// MT3006 - LABORATORIO 3: BASE PARTE 2
// ================================================================================
// El siguiente script presenta el funcionamiento base del TinyS3 del Pololu 3Pi+,
// el cual usted deberá modificar para que pueda emplear la información de la 
// OpenMV Cam para realizar el control basado en visión.
// NOTA 1: lea cuidadosamente los comentarios en el código para ubicar las 
// secciones a modificar y las que *** NO DEBE MODIFICAR NI QUITAR ***.
// NOTA 2: la decodificación de la información enviada por la OpenMV Cam deberá
// hacerse mediante la función sscanf.
// ================================================================================
// Dependencias | librerías   *** NO DEBE MODIFICAR NI QUITAR ***
// ================================================================================
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <UMS3.h>
#include <tinycbor.h>


// ================================================================================
// Variables
// ================================================================================
// Placa y pines    *** NO DEBE MODIFICAR NI QUITAR ***
UMS3 board;
static const uint8_t pin_servo1 = 1u;
static const uint8_t pin_servo2 = 2u;
static const uint8_t pin_servo3 = 3u;
static const uint8_t pin_robot_switch = 4u;
static const uint8_t pin_bat_level = 5u;
static const uint8_t pin_camera_handshake = 9u;
static const uint8_t pin_tx_to_robot = 7u;
static const uint8_t pin_rx_to_robot = 8u;
static const uint8_t pin_tx_to_camera = 35u;
static const uint8_t pin_rx_to_camera = 37u;

// Comunicación CBOR-UART con el 32u4 del Pololu    *** NO DEBE MODIFICAR NI QUITAR ***
uint8_t uart_send_buffer[32] = {0}; // buffer CBOR
static const unsigned int control_time_ms = 100u; // período de muestreo del control
volatile float phi_ell = 0; // en rpm
volatile float phi_r = 0; // en rpm

// Comunicación JSON-UART con la cámara    *** NO DEBE MODIFICAR NI QUITAR ***
#define JSON_BUFLEN (512u)
volatile char recv_buf[JSON_BUFLEN];
volatile char data_buf[JSON_BUFLEN];
unsigned buf_idx = 0;
char camdata;
bool b_cam_data_parse = false;

// ElegantOTA   *** NO DEBE MODIFICAR NI QUITAR ***
const char* ssid = "Robotat";
const char* password = "iemtbmcit116";
WebServer server(80);
unsigned long ota_progress_millis = 0;

// Handles de tasks   *** NO DEBE MODIFICAR NI QUITAR ***
TaskHandle_t encode_send_wheel_speeds_handle;
TaskHandle_t visual_servoing_handle;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// COLOCAR AQUÍ LAS VARIABLES GLOBALES DEL USUARIO (DE SER REQUERIDO)
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Estado de cámara recibido por UART (OpenMV)
volatile int cx = 0;
volatile int cy = 0;
volatile int area = 0;

// Geometría del robot (mm)
static constexpr float R_MM = 32.0f;                 // radio de rueda (Ø32 mm -> 16)
static constexpr float L_MM = (96.0f - 2.0f*6.8f);   // distancia entre centros de rueda

// Cámara
static constexpr int CAM_W = 320;
static constexpr int CAM_H = 240;

// --- Control IBVS (px / px^2), espejo de tu Webots ---
// ω = -k_omega * (cx - u0)   [rad/s ; u en píxeles]
// v =  k_v     * (A_target - area)  limitado por V_CAP, y v=0 si sobrepasó la meta
static constexpr float k_omega = 0.015f;        // [rad/(s·px)]
static constexpr float k_v     = 0.12f;         // [mm/(s·px^2)]
static constexpr float TARGET_FRAC = 0.03f;     // 3% del frame
static constexpr float V_CAP       = 250.0f;    // límite lineal del cuerpo [mm/s]
static constexpr float W_SEARCH    = 0.6f;      // rad/s cuando no hay blob

// Suavizados / límites adicionales
static constexpr float W_MAX = 1.2f;            // límite de giro (rad/s) para robustez
static constexpr float RPM_MAX = 120.0f;        // saturación inicial segura (subir luego)
static constexpr float MYTWO_PI = 6.28318530718f;

// Utilidades
inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }
inline float rads_to_rpm(float w_rad_s) { return w_rad_s * (60.0f / MYTWO_PI); }
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// ================================================================================
// Funciones | Métodos
// ================================================================================
inline void
cbor_encode_uart_send()   // *** NO DEBE MODIFICAR NI QUITAR ***
{
  TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
  TinyCBOR.Encoder.create_array(2);
  TinyCBOR.Encoder.encode_float(phi_ell);
  TinyCBOR.Encoder.encode_float(phi_r);
  TinyCBOR.Encoder.close_container();
  Serial2.write(TinyCBOR.Encoder.get_buffer(), TinyCBOR.Encoder.get_buffer_size()); 
}

void 
onOTAStart()    // *** NO DEBE MODIFICAR NI QUITAR ***
{
  // Log when OTA has started
  Serial.println("OTA update started!");
  // Custom OTA code
  phi_ell = 0;
  phi_r = 0;
  cbor_encode_uart_send();
  digitalWrite(pin_robot_switch, LOW); // Motores apagados
  // Se deshabilitan las tareas que empleen periféricos rápidos
  vTaskDelete(encode_send_wheel_speeds_handle); 
  vTaskDelete(visual_servoing_handle); 
}

void 
onOTAProgress(size_t current, size_t final)   // *** NO DEBE MODIFICAR NI QUITAR ***
{
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) 
  {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void 
onOTAEnd(bool success)    // *** NO DEBE MODIFICAR NI QUITAR ***
{
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // Custom OTA code
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// COLOCAR AQUÍ LAS FUNCIONES DEL USUARIO (DE SER REQUERIDO)
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// (no requerido)
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// ================================================================================
// Tasks
// ================================================================================
void
encode_send_wheel_speeds_task(void * p_params)    // *** NO DEBE MODIFICAR NI QUITAR ***
{
  TickType_t last_control_time;
  const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);

  // Tiempo actual
  last_control_time = xTaskGetTickCount();

  while(1)
  {
    // Se espera hasta que se cumpla el período de muestreo
    vTaskDelayUntil(&last_control_time, control_freq_ticks);
    cbor_encode_uart_send();    
  }
}

void
blinky_task(void * p_params)    // *** NO DEBE MODIFICAR NI QUITAR ***
{
  
  while(1) 
  {
    board.setPixelColor(50, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    board.setPixelColor(50, 0, 50);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// VISUAL SERVOING A MODIFICAR
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// En esta sección de código debe hacerse:
// 1. La recepción (ya está implementada) y decodificación (hay que implementarla) 
//    de data de la cámara.
// 2. Calcular las velocidades lineal y angular del robot mediante visual servoing.
// 3. Transformar las velocidades del paso anterior a las velocidades de las 
//    ruedas en rpm (usar variables distintas a phi_ell y phi_r).
// 4. Saturar las velocidades máximas de las ruedas por seguridad.
// 5. Enviar las velocidades a las ruedas (ahora sí se emplean las variables
//    phi_ell y phi_r).
// OJO: dentro del código suministrado se le deja un ejemplo de cómo hacer 
//      correctamente los delays, en caso de requerirlos.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void
visual_servoing_task(void * p_params)
{
  // Temporales de control
  float v_mm_s = 0.0f;   // velocidad lineal del cuerpo [mm/s]
  float w_rad_s = 0.0f;  // velocidad angular del cuerpo [rad/s]
  
  while(1) 
  {
    // 1.a) Recepción y lectura de la data formateada de la cámara.
    if(Serial1.available())
    {
      camdata = Serial1.read();
      if(camdata == '\n')
      {
        recv_buf[buf_idx] = '\0';
        strcpy((char *) data_buf, (char *) recv_buf);
        Serial.println((char *) data_buf);
        buf_idx = 0;
        b_cam_data_parse = true;
      }
      else
      {
        if (buf_idx < (JSON_BUFLEN - 1)) recv_buf[buf_idx++] = camdata; // guard contra overflow
        else buf_idx = 0;
      }
    }
    else
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    if(b_cam_data_parse)
    {
      // 1.b) Decodificar empleando la función sscanf (la data a usar está en el string data_buf)
      // Esperado (mínimo): {"cx":192,"cy":147,"area":360}
      int t_cx=0, t_cy=0, t_area=0;
      if (sscanf((char*)data_buf, "{\"cx\":%d,\"cy\":%d,\"area\":%d}", &t_cx, &t_cy, &t_area) == 3) {
          cx = t_cx; cy = t_cy; area = t_area;
      } else {
          Serial.println("Error al parsear con sscanf");
      }

      // 2) Calcular v y w (IBVS, px/px^2) como en Webots:
      //    w = -k_omega * (cx - u0)     [e_u en píxeles, NO normalizado]
      //    v =  k_v     * (A_target - area) limitado por V_CAP; si área >= target, v=0
      const float u0 = 0.5f * CAM_W;
      const float A_target = TARGET_FRAC * (float(CAM_W) * float(CAM_H));

      float e_u_px = float(cx) - u0;                 // error horizontal en px
      w_rad_s = -k_omega * e_u_px;                   // giro en rad/s
      w_rad_s = clampf(w_rad_s, -W_MAX, W_MAX);     // límite de giro (robustez)

      float eA_px2 = A_target - float(area);         // error de área en px^2
      if (area > 0 && cx > 0 && cy > 0) {
        if (eA_px2 > 0.0f) v_mm_s = clampf(k_v * eA_px2, 0.0f, V_CAP);
        else               v_mm_s = 0.0f;            // ya alcanzó ~3%
      } else {
        // Sin blob: búsqueda sin avance
        v_mm_s = 0.0f;
        w_rad_s = W_SEARCH;
      }

      // 3) Transformar a velocidades de ruedas (mm/s)
      //    v_L = v - (L/2)*w,  v_R = v + (L/2)*w
      const float vL_mm_s = v_mm_s - 0.5f * L_MM * w_rad_s;
      const float vR_mm_s = v_mm_s + 0.5f * L_MM * w_rad_s;

      // 4) mm/s -> rpm y saturación por rueda
      const float wL_rad_s = vL_mm_s / R_MM;
      const float wR_rad_s = vR_mm_s / R_MM;

      float rpmL = rads_to_rpm(wL_rad_s);
      float rpmR = rads_to_rpm(wR_rad_s);

      rpmL = clampf(rpmL, -RPM_MAX, RPM_MAX);
      rpmR = clampf(rpmR, -RPM_MAX, RPM_MAX);

      // 5) Enviar a ruedas (CBOR)
      // Nota: si algún lado está invertido en tu 3Pi+, invierte el signo del correspondiente.
      phi_ell = rpmL;
      phi_r   = rpmR;

      b_cam_data_parse = false; // *** NO DEBE MODIFICAR NI QUITAR ***
    }

    // (higiene) evita busy-wait cuando hay flujo continuo
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// ================================================================================
// Setup y loop
// ================================================================================
void 
setup(void) 
{
  // Placa y pines    *** NO DEBE MODIFICAR NI QUITAR ***
  board.begin();
  board.setPixelPower(true);
  board.setPixelColor(0, 0, 0);  
  pinMode(pin_servo1, OUTPUT);
  pinMode(pin_servo2, OUTPUT);
  pinMode(pin_servo3, OUTPUT);
  pinMode(pin_robot_switch, OUTPUT);
  pinMode(pin_bat_level, INPUT);
  pinMode(pin_camera_handshake, OUTPUT);
  
  digitalWrite(pin_camera_handshake, HIGH); // Sin solicitar datos
  digitalWrite(pin_robot_switch, LOW); // Motores apagados
  
  // Coms   *** NO DEBE MODIFICAR NI QUITAR ***
  Serial.begin(115200); 

  Serial1.begin(115200, SERIAL_8N1, pin_rx_to_camera, pin_tx_to_camera); // Camera
  Serial2.begin(115200, SERIAL_8N1, pin_rx_to_robot, pin_tx_to_robot); // Robot
  TinyCBOR.init(); 

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Se espera hasta establecer conexión    *** NO DEBE MODIFICAR NI QUITAR ***
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  board.setPixelColor(50, 0, 50); // Se enciende el LED para validar conexión
  digitalWrite(pin_robot_switch, HIGH); // Se encienden los motores

  server.on("/", []() 
  {
    server.send(200, "text/plain", "This is a Pololu robotic agent connected to the Robotat.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // COLOCAR AQUÍ EL CÓDIGO DE SETUP DEL USUARIO (DE SER REQUERIDO)
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Creación de tasks 
  xTaskCreate(encode_send_wheel_speeds_task, "encode_send_wheel_speeds_task", 1024*2, NULL, configMAX_PRIORITIES-1, &encode_send_wheel_speeds_handle);
  xTaskCreate(visual_servoing_task, "visual_servoing_task", 1024*4, NULL, configMAX_PRIORITIES-2, &visual_servoing_handle);
  xTaskCreate(blinky_task, "blinky_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
}

void 
loop(void) 
{
  server.handleClient();
  ElegantOTA.loop();
}
