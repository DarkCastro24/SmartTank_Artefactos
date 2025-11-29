/*
  SMARTANK + MQTT (ESP32) – Versión JSON para EMQX Web Dashboard
  Fecha: 17/11/2025

  Publica TODO el estado del tanque como JSON en:
      smartank/estado

  Compatible con dashboard:
      wss://broker.emqx.io:8084/mqtt
*/

#include <WiFi.h>
#include <PubSubClient.h>

//******** WIFI ********
const char *ssid   = "Los_Castro";
const char *passwd = "Extreme8000";

//******** MQTT (EMQX – Puerto normal TCP) ********
char *server = "broker.emqx.io";
int port = 1883;

const char *mqtt_user     = "kstro";   // opcional
const char *mqtt_password = "2002";    // opcional

// Topic principal JSON
const char *TOPIC_JSON = "smartank/estado";

//******** CLIENTES ********
WiFiClient wlanclient;
PubSubClient mqttClient(wlanclient);

//******** Tiempo para mostrar estado wifi ********
unsigned long interval = 30000;
unsigned long previousMillis = 0;

//=====================================================================
//                    CONFIGURACIÓN DE PINES
//=====================================================================
#define PIN_TRIG            5
#define PIN_ECHO            18

#define PIN_PH              34
#define PIN_TDS             35

#define PIN_BOMBA_DRENAJE   26
#define PIN_BOMBA_LLENADO   27

#define PIN_LED_VERDE       14
#define PIN_LED_AMARILLO    12
#define PIN_LED_ROJO        13

//=====================================================================
//                    PARÁMETROS Y CALIBRACIONES
//=====================================================================

float VELOCIDAD_SONIDO = 0.0343 / 2.0; 
float MAX_ALTURA       = 30.0;

float DESPLAZAMIENTO_PH = 0.00;
float PENDIENTE_PH      = -0.18;

float FACTOR_TDS = 0.5;

int TDS_BAJO = 250;
int TDS_ALTO = 800;

float UMBRAL_LLENADO = 90.0;

//=====================================================================
//                VALORES DEFAULT PARA ERRORES
//=====================================================================
const float DEFAULT_DISTANCIA = -1.0;
const float DEFAULT_NIVEL     = 0.0;
const float DEFAULT_PH        = 7.00;
const float DEFAULT_TDS       = 0.0;

//=====================================================================
//           MODO MANUAL / AUTOMÁTICO
//=====================================================================
bool MODO_MANUAL         = false;
bool EST_MANUAL_LLENADO  = false;
bool EST_MANUAL_DRENAJE  = false;

void establecerModoManual(bool x) {
  MODO_MANUAL = x;
  Serial.println(x ? "Modo MANUAL" : "Modo AUTOMÁTICO");
}

void establecerEstadoManualLlenado(bool x) {
  EST_MANUAL_LLENADO = x;
  Serial.println(x ? "Llenado ON" : "Llenado OFF");
}

void establecerEstadoManualDrenaje(bool x) {
  EST_MANUAL_DRENAJE = x;
  Serial.println(x ? "Drenaje ON" : "Drenaje OFF");
}

//=====================================================================
//                       FUNCIONES DE LECTURA
//=====================================================================

float leerUltrasonico() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duracion = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duracion == 0) return -1.0;

  return duracion * VELOCIDAD_SONIDO;
}

float convertirVoltajeATDS(float v) {
  return (133.42 * v*v*v - 255.86 * v*v + 857.39 * v) * FACTOR_TDS;
}

float leerPH() {
  int muestras = 15;
  unsigned long acum = 0;
  for (int i = 0; i < muestras; i++) {
    acum += analogRead(PIN_PH);
    delay(10);
  }
  float adc = acum / (float)muestras;
  float v = adc * (3.3 / 4095.0);
  float ph = 7.0 + DESPLAZAMIENTO_PH + (2.5 - v) / PENDIENTE_PH;
  return ph;
}

float leerTDS() {
  int muestras = 15;
  unsigned long acum = 0;
  for (int i = 0; i < muestras; i++) {
    acum += analogRead(PIN_TDS);
    delay(10);
  }
  float adc = acum / (float)muestras;
  float v = adc * (3.3 / 4095.0);
  return convertirVoltajeATDS(v);
}

//=====================================================================
//                      INDICADORES LED
//=====================================================================
void actualizarIndicadores(float porcentaje) {
  if (porcentaje > 85.0) {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  } else if (porcentaje > 25.0) {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, HIGH);
    digitalWrite(PIN_LED_ROJO, LOW);
  } else {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, HIGH);
  }
}

//=====================================================================
//                CALLBACK MQTT (COMANDOS OPCIONALES)
//=====================================================================
void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  // No usaremos control bidireccional por ahora
}

//=====================================================================
//             RECONNECT MQTT
//=====================================================================
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Conectando MQTT (EMQX)...");
    if (mqttClient.connect("ESP32_SMARTANK", mqtt_user, mqtt_password)) {
      Serial.println("MQTT Conectado!");
    } else {
      Serial.print("Error MQTT: ");
      Serial.println(mqttClient.state());
      delay(3000);
    }
  }
}

//=====================================================================
//                ESTADO DEL WIFI
//=====================================================================
void printWifiStatus() {
  unsigned long now = millis();
  if (now - previousMillis >= interval) {
    Serial.println("\n----------------------------------------");
    Serial.println("Estado WIFI:");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
    Serial.println("----------------------------------------");
    previousMillis = now;
  }
}

//=====================================================================
//                         SETUP
//=====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_BOMBA_DRENAJE, OUTPUT);
  pinMode(PIN_BOMBA_LLENADO, OUTPUT);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PH,  ADC_11db);
  analogSetPinAttenuation(PIN_TDS, ADC_11db);

  // WIFI
  WiFi.begin(ssid, passwd);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nWiFi conectado!");

  // MQTT
  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqttCallback);

  reconnect();  
}

//=====================================================================
//                         LOOP
//=====================================================================
void loop() {

  printWifiStatus();

  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();

  //------ Lectura de sensores ------
  float distancia = leerUltrasonico();
  float porcentaje = 0.0;

  if (distancia < 0) {
    distancia = DEFAULT_DISTANCIA;
    porcentaje = DEFAULT_NIVEL;
  } else {
    float altura = MAX_ALTURA - distancia;
    if (altura < 0) altura = 0;
    if (altura > MAX_ALTURA) altura = MAX_ALTURA;
    porcentaje = (altura / MAX_ALTURA) * 100.0;
  }

  float altura = MAX_ALTURA - distancia;
  if (altura < 0) altura = 0;
  if (altura > MAX_ALTURA) altura = MAX_ALTURA;

  float ph = leerPH();
  if (isnan(ph) || ph < 0 || ph > 14) ph = DEFAULT_PH;

  float tds = leerTDS();
  if (isnan(tds) || tds < 0 || tds > 2000) tds = DEFAULT_TDS;

  actualizarIndicadores(porcentaje);

  // Determinar LED
  String ledStr;
  if (porcentaje > 85.0) ledStr = "VERDE";
  else if (porcentaje > 25.0) ledStr = "AMARILLO";
  else ledStr = "ROJO";

  //-------- PUBLICAR JSON --------
  String json = "{";
  json += "\"distancia\":" + String(distancia, 2) + ",";
  json += "\"altura\":" + String(altura, 2) + ",";
  json += "\"porcentaje\":" + String(porcentaje, 0) + ",";
  json += "\"ph\":" + String(ph, 2) + ",";
  json += "\"tds\":" + String(tds, 0) + ",";
  json += "\"bomba_llenado\":" + String(digitalRead(PIN_BOMBA_LLENADO)==HIGH?"true":"false") + ",";
  json += "\"bomba_drenaje\":" + String(digitalRead(PIN_BOMBA_DRENAJE)==HIGH?"true":"false") + ",";
  json += "\"modo_manual\":" + String(MODO_MANUAL ? "true" : "false") + ",";
  json += "\"led_estado\":\"" + ledStr + "\"";
  json += "}";

  mqttClient.publish(TOPIC_JSON, json.c_str());

  Serial.println("Publicado JSON: ");
  Serial.println(json);
  Serial.println("----------------------------------------");

  delay(1000);
}
