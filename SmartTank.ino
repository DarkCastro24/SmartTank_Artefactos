/*****  CODIGO FINAL PROYECTO SMARTANK + MQTT 
    PLACA UTILIZADA : ESP32 CON ESP32 DEV MODULE
    ULTIMA FECHA DE MODIFICACION: 29/11/2025  *****/

#include <WiFi.h>
#include <PubSubClient.h>

/***** WiFi (names kept in English) *****/
const char *ssid = "Los_Castro";
const char *wifiPassword = "Extreme8000";

/***** MQTT (names kept in English) *****/
char *mqttServer = "broker.emqx.io";
int mqttPort = 1883;
const char *mqttUser     = "kstro";
const char *mqttPassword = "2002";
const char *TOPIC_JSON      = "smartank/estado";
const char *TOPIC_CONTROL   = "smartank/control";

/***** Network clients (kept in English) *****/
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/***** Conexion de pines *****/
#define PIN_TRIG            5
#define PIN_ECHO            18
#define PIN_PH              34
#define PIN_TDS             35
#define PIN_BOMBA_LLENADO   22
#define PIN_BOMBA_DRENAJE   23
#define PIN_LED_VERDE       14
#define PIN_LED_AMARILLO    12
#define PIN_LED_ROJO        13

/***** Relés (activo en LOW) *****/
const int RELE_ON  = LOW;
const int RELE_OFF = HIGH;

/***** Parametros fisicos o de operación *****/
float velocidadSonido = 0.0343 / 2.0;
float alturaMaxima = 12.0;

/***** Calibración PH/TDS *****/
float desplazamientoPh = 0.00;
float pendientePh = -0.18;
float factorTds = 0.5;

/***** Umbrales / Tolerancia *****/
float umbralLlenado = 75.0;
const float porcentajeTolerancia = 10.0;
const int lecturasEstableRequeridas = 5;
int contadorEstable = 0;

/***** TDS / auto-drenaje *****/
const float umbralDrenajeTdsPpm = 300.0;

/***** Duraciones del patrón TDS (ms) *****/
const unsigned long TDS_DRENAR_MS = 5000UL;
const unsigned long TDS_LLENAR_MS  = 4500UL;

/***** Valores por defecto / fallos *****/
const float DISTANCIA_DEFAULT = -1.0;
const float NIVEL_DEFAULT     = 0.0;
const float PH_DEFAULT        = 7.00;
const float TDS_DEFAULT       = 0.0;

/***** Modos y flags *****/
bool modoManual = true;
bool estadoManualLlenado = false;
bool estadoManualDrenaje = false;
bool modoDrenajeTds = false;

/***** Ultrasonido: muestras y cache *****/
#define ULTRA_MUESTRAS 5
const unsigned long ULTIMA_LECTURA_MAX_ANTIGUA_MS = 5000UL;
float ultimaDistanciaValida = DISTANCIA_DEFAULT;
unsigned long ultimoTiempoValido = 0;

/***** Estado para publicación / UI *****/
float porcentajeMostrado = 0.0;

/***** Temporizadores / impresión *****/
unsigned long intervaloImpresionWifiMs = 30000;
unsigned long ultimoMillisWifi = 0;
const unsigned long INTERVALO_IMPRESION_ESTADO_MS = 5000;
unsigned long ultimoPrintMillis = 0;

/***** FUNCIONES *****/

float leerUltrasonico() {
  long tiempos[ULTRA_MUESTRAS];
  int contValidas = 0;

  for (int i = 0; i < ULTRA_MUESTRAS; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duracion = pulseIn(PIN_ECHO, HIGH, 30000);
    if (duracion > 0) {
      tiempos[contValidas++] = duracion;
    }
    delay(30);
  }

  unsigned long ahora = millis();

  if (contValidas == 0) {
    if (ultimaDistanciaValida >= 0 && (ahora - ultimoTiempoValido) <= ULTIMA_LECTURA_MAX_ANTIGUA_MS) {
      return ultimaDistanciaValida;
    } else {
      return -1.0;
    }
  }

  for (int i = 0; i < contValidas - 1; i++) {
    for (int j = i + 1; j < contValidas; j++) {
      if (tiempos[j] < tiempos[i]) {
        long tmp = tiempos[i];
        tiempos[i] = tiempos[j];
        tiempos[j] = tmp;
      }
    }
  }

  long duracionMediana;
  if (contValidas % 2 == 1) {
    duracionMediana = tiempos[contValidas / 2];
  } else {
    duracionMediana = (tiempos[contValidas / 2 - 1] + tiempos[contValidas / 2]) / 2;
  }

  float distancia = duracionMediana * velocidadSonido;

  if (distancia < 0) distancia = 0;
  if (distancia > alturaMaxima) distancia = alturaMaxima;

  ultimaDistanciaValida = distancia;
  ultimoTiempoValido = ahora;

  return distancia;
}

float convertirVoltajeATds(float v) {
  return (133.42 * v*v*v - 255.86 * v*v + 857.39 * v) * factorTds;
}

float leerPh() {
  int muestras = 15;
  unsigned long suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(PIN_PH);
    delay(10);
  }
  float adc = suma / (float)muestras;
  float v = adc * (3.3 / 4095.0);
  return 7.0 + desplazamientoPh + (2.5 - v) / pendientePh;
}

float leerTds() {
  int muestras = 15;
  unsigned long suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(PIN_TDS);
    delay(10);
  }
  float adc = suma / (float)muestras;
  float v = adc * (3.3 / 4095.0);
  return convertirVoltajeATds(v);
}

void actualizarIndicadores(float porcentaje) {
  if (porcentaje >= 70.0) {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  } else if (porcentaje > 30.0) {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, HIGH);
    digitalWrite(PIN_LED_ROJO, LOW);
  } else {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, HIGH);
  }
}

String clasificarTds(float tds) {
  if (tds <= 150.0) return "muy buena";
  if (tds <= 200.0) return "buena";
  if (tds <= 250.0) return "regular";
  if (tds <= 350.0) return "mala";
  return "muy mala";
}

void publicarEstado() {
  float distancia = leerUltrasonico();
  float altura = (distancia < 0) ? 0 : alturaMaxima - distancia;
  if (altura < 0) altura = 0;
  if (altura > alturaMaxima) altura = alturaMaxima;
  float porcentajeMedido = (altura / alturaMaxima) * 100.0;

  porcentajeMostrado = porcentajeMedido;
  if (porcentajeMedido >= 50.0) {
    porcentajeMostrado = porcentajeMedido + 7.0;
    if (porcentajeMostrado > 100.0) porcentajeMostrado = 100.0;
  }

  float ph = leerPh();
  if (isnan(ph) || ph < 0 || ph > 14) ph = PH_DEFAULT;
  float tds = leerTds();
  if (isnan(tds) || tds < 0 || tds > 2000) tds = TDS_DEFAULT;

  bool bombaL = (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON);
  bool bombaD = (digitalRead(PIN_BOMBA_DRENAJE) == RELE_ON);

  String tdsCal = clasificarTds(tds);

  String json = "{";
  json += "\"distancia\":" + String(distancia, 2) + ",";
  json += "\"altura\":" + String(altura, 2) + ",";
  json += "\"porcentaje\":" + String(porcentajeMostrado, 0) + ",";
  json += "\"porcentaje_medido\":" + String(porcentajeMedido, 1) + ",";
  json += "\"ph\":" + String(ph, 2) + ",";
  json += "\"tds\":" + String(tds, 0) + ",";
  json += "\"tds_calidad\":\"" + tdsCal + "\",";
  json += "\"bomba_llenado\":" + String(bombaL ? "true" : "false") + ",";
  json += "\"bomba_drenaje\":" + String(bombaD ? "true" : "false") + ",";
  json += "\"modo_manual\":" + String(modoManual ? "true" : "false");
  json += "}";

  mqttClient.publish(TOPIC_JSON, json.c_str());
}

void establecerModoManual(bool x) {
  modoManual = x;
  Serial.print(">>> Modo cambiado a: ");
  Serial.println(modoManual ? "MANUAL" : "AUTOMATICO");

  if (modoManual) {
    estadoManualLlenado = false;
    estadoManualDrenaje = false;
    digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
    modoDrenajeTds = false;
    contadorEstable = 0;
  } else {
    estadoManualLlenado = false;
    estadoManualDrenaje = false;
    contadorEstable = 0;
  }

  if (mqttClient.connected()) publicarEstado();
}

void establecerEstadoManualLlenado(bool x) {
  estadoManualLlenado = x;
  Serial.print(">>> Llenado manual (flag) set a: ");
  Serial.println(x ? "ON" : "OFF");
  if (!estadoManualLlenado) {
    digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  }
  if (mqttClient.connected()) publicarEstado();
}

void establecerEstadoManualDrenaje(bool x) {
  estadoManualDrenaje = x;
  Serial.print(">>> Drenaje manual (flag) set a: ");
  Serial.println(x ? "ON" : "OFF");
  if (estadoManualDrenaje) {
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
  } else {
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
  }
  if (mqttClient.connected()) publicarEstado();
}

void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  String topic = String(topicChar);
  String cmd = "";
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  Serial.print("MQTT recibido en ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(cmdUpper);

  if (topic == String(TOPIC_CONTROL)) {
    if (cmdUpper == "ON" || cmdUpper == "MODE:ON" || cmdUpper == "MODE_ON") {
      establecerModoManual(true);
    } else if (cmdUpper == "OFF" || cmdUpper == "MODE:OFF" || cmdUpper == "MODE_OFF") {
      establecerModoManual(false);
    } else if (cmdUpper == "PUMP_ON" || cmdUpper == "PUMP:ON" || cmdUpper == "LLENADO_ON") {
      if (!modoManual) {
        Serial.println("PUMP_ON recibido: forzando MODO_MANUAL");
        establecerModoManual(true);
      }
      estadoManualLlenado = true;
      Serial.println("EST_MANUAL_LLENADO = true (orden PUMP_ON).");
      if (mqttClient.connected()) publicarEstado();
    } else if (cmdUpper == "PUMP_OFF" || cmdUpper == "PUMP:OFF" || cmdUpper == "LLENADO_OFF") {
      estadoManualLlenado = false;
      digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
      Serial.println("PUMP_OFF recibido: bomba apagada (manual).");
      if (mqttClient.connected()) publicarEstado();
    } else if (cmdUpper == "DRENAR_ON" || cmdUpper == "DREN:ON") {
      if (!modoManual) {
        Serial.println("DRENAR_ON recibido: forzando MODO_MANUAL");
        establecerModoManual(true);
      }
      estadoManualDrenaje = true;
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
      Serial.println("DRENAR_ON: drenaje activado (manual).");
      if (mqttClient.connected()) publicarEstado();
    } else if (cmdUpper == "DRENAR_OFF" || cmdUpper == "DREN:OFF") {
      estadoManualDrenaje = false;
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
      Serial.println("DRENAR_OFF: drenaje apagado (manual).");
      if (mqttClient.connected()) publicarEstado();
    } else {
      Serial.println("Comando no reconocido.");
    }
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Conectando MQTT (EMQX)...");
    if (mqttClient.connect("ESP32_SMARTANK", mqttUser, mqttPassword)) {
      Serial.println("MQTT Conectado!");
      mqttClient.subscribe(TOPIC_CONTROL);
      publicarEstado();
    } else {
      Serial.print("Error MQTT: ");
      Serial.println(mqttClient.state());
      delay(3000);
    }
  }
}

void printWifiStatus() {
  unsigned long ahora = millis();
  if (ahora - ultimoMillisWifi >= intervaloImpresionWifiMs) {
    Serial.println("\n----------------------------------------");
    Serial.println("Estado WIFI:");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.println("----------------------------------------");
    ultimoMillisWifi = ahora;
  }
}

void waitWithMqtt(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    if (!mqttClient.connected()) reconnect();
    mqttClient.loop();
    delay(50);
  }
}

void ejecutarCicloDrenajeTds() {
  if (modoManual) return;

  Serial.println(">>> Iniciando ciclo TDS: drenaje 5s -> llenado 4.5s");

  digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  delay(20);
  digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
  if (mqttClient.connected()) publicarEstado();
  waitWithMqtt(TDS_DRENAR_MS);
  digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
  if (mqttClient.connected()) publicarEstado();
  delay(150);

  digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
  if (mqttClient.connected()) publicarEstado();
  waitWithMqtt(TDS_LLENAR_MS);
  digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  if (mqttClient.connected()) publicarEstado();

  Serial.println(">>> Fin de ciclo TDS (se evaluará TDS nuevamente).");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_BOMBA_LLENADO, OUTPUT);
  pinMode(PIN_BOMBA_DRENAJE, OUTPUT);

  digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PH,  ADC_11db);
  analogSetPinAttenuation(PIN_TDS, ADC_11db);

  WiFi.begin(ssid, wifiPassword);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nWiFi conectado!");

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  reconnect();

  ultimoPrintMillis = millis();
}

void loop() {
  printWifiStatus();

  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();

  float distancia = leerUltrasonico();
  float porcentajeMedido = 0.0;
  float altura = 0.0;

  if (distancia < 0) {
    distancia = DISTANCIA_DEFAULT;
    porcentajeMedido = NIVEL_DEFAULT;
    altura = 0.0;
  } else {
    altura = alturaMaxima - distancia;
    if (altura < 0) altura = 0;
    if (altura > alturaMaxima) altura = alturaMaxima;
    porcentajeMedido = (altura / alturaMaxima) * 100.0;
  }

  float ph = leerPh();
  if (isnan(ph) || ph < 0 || ph > 14) ph = PH_DEFAULT;

  float tds = leerTds();
  if (isnan(tds) || tds < 0 || tds > 2000) tds = TDS_DEFAULT;

  actualizarIndicadores(porcentajeMedido);

  unsigned long ahora = millis();

  if (!modoManual) {
    if (tds > umbralDrenajeTdsPpm && distancia >= 0) {
      modoDrenajeTds = true;
      Serial.println(">>> TDS alto detectado en automático -> entrando en MODO_DRENAGE_TDS.");
      while (!modoManual) {
        float tdsAhora = leerTds();
        if (isnan(tdsAhora)) tdsAhora = tds;
        Serial.print("   > TDS actual (antes de ciclo): ");
        Serial.println(tdsAhora);
        if (tdsAhora <= umbralDrenajeTdsPpm) {
          Serial.println("   > TDS ya bajo o igual al umbral; saliendo del modo drenaje TDS.");
          modoDrenajeTds = false;
          break;
        }
        ejecutarCicloDrenajeTds();
        waitWithMqtt(300);
        float tdsAfter = leerTds();
        Serial.print("   > TDS después del ciclo: ");
        Serial.println(tdsAfter);
        if (tdsAfter <= umbralDrenajeTdsPpm) {
          Serial.println("   > TDS corregido. Finalizando MODO_DRENAGE_TDS.");
          modoDrenajeTds = false;
          break;
        } else {
          Serial.println("   > TDS aún alto -> repetir ciclo (siempre en automático).");
        }
      }
    }

    if (!modoDrenajeTds) {
      if (distancia >= 0) {
        float onThreshold = umbralLlenado - porcentajeTolerancia;
        float offThreshold = umbralLlenado;

        if (porcentajeMedido < onThreshold) {
          contadorEstable++;
          if (contadorEstable >= lecturasEstableRequeridas) {
            if (digitalRead(PIN_BOMBA_LLENADO) != RELE_ON) {
              digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
              Serial.println("AUTO: Arrancando bomba (condición estable por debajo del onThreshold).");
            }
            contadorEstable = lecturasEstableRequeridas;
          }
        } else {
          contadorEstable = 0;
        }

        if (porcentajeMedido >= offThreshold) {
          if (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON) {
            digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
            Serial.println("AUTO: Apagando bomba (porcentaje_medido >= offThreshold).");
          }
          contadorEstable = 0;
        }
      } else {
        digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
        contadorEstable = 0;
      }
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
    }
  } else {
    if (estadoManualLlenado) {
      if (porcentajeMedido >= umbralLlenado) {
        digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
        estadoManualLlenado = false;
        Serial.println("Bomba apagada automáticamente por alcanzar umbral (modo manual).");
        if (mqttClient.connected()) publicarEstado();
      } else {
        digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
      }
    } else {
      digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
    }

    if (estadoManualDrenaje) {
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
    } else {
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
    }
  }

  bool bombaLEstado = (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON);
  bool bombaDEstado = (digitalRead(PIN_BOMBA_DRENAJE) == RELE_ON);

  String tdsCal = clasificarTds(tds);

  porcentajeMostrado = porcentajeMedido;
  if (porcentajeMedido >= 45.0) {
    porcentajeMostrado = porcentajeMedido + 7.5;
    if (porcentajeMostrado > 100.0) porcentajeMostrado = 100.0;
  }

  String json = "{";
  json += "\"distancia\":" + String(distancia, 2) + ",";
  json += "\"altura\":" + String(altura, 2) + ",";
  json += "\"porcentaje\":" + String(porcentajeMostrado, 0) + ",";
  json += "\"porcentaje_medido\":" + String(porcentajeMedido, 1) + ",";
  json += "\"ph\":" + String(ph, 2) + ",";
  json += "\"tds\":" + String(tds, 0) + ",";
  json += "\"tds_calidad\":\"" + tdsCal + "\",";
  json += "\"bomba_llenado\":" + String(bombaLEstado ? "true" : "false") + ",";
  json += "\"bomba_drenaje\":" + String(bombaDEstado ? "true" : "false") + ",";
  json += "\"modo_manual\":" + String(modoManual ? "true" : "false");
  json += "}";

  if (mqttClient.connected()) mqttClient.publish(TOPIC_JSON, json.c_str());

  if (ahora - ultimoPrintMillis >= INTERVALO_IMPRESION_ESTADO_MS) {
    ultimoPrintMillis = ahora;
    Serial.println("==== MOSTRANDO ESTADO DEL SISTEMA ====");
    Serial.print("Modo actual: ");
    Serial.println(modoManual ? "MANUAL" : "AUTOMATICO");
    Serial.print("Bomba llenado: ");
    Serial.println(bombaLEstado ? "ENCENDIDA" : "APAGADA");
    Serial.print("Bomba drenaje: ");
    Serial.println(bombaDEstado ? "ENCENDIDA" : "APAGADA");
    Serial.print("Distancia (cm): ");
    Serial.println(distancia, 2);
    Serial.print("Porcentaje medido: ");
    Serial.println(porcentajeMedido, 1);
    Serial.print("Porcentaje mostrado (MQTT): ");
    Serial.println(porcentajeMostrado, 1);
    Serial.print("TDS (ppm): ");
    Serial.println(tds);
    Serial.print("TDS calidad: ");
    Serial.println(tdsCal);
    Serial.println("JSON enviado:");
    Serial.println(json);
    Serial.println("================================");
  }

  delay(200);
}
