/*****  CODIGO RESPALDO FINAL PROYECTO SMARTANK + MQTT
    PLACA UTILIZADA : ESP32 CON ESP32 DEV MODULE
    ULTIMA FECHA DE MODIFICACION: 30/11/2025  *****/

#include <WiFi.h>
#include <PubSubClient.h>

/***** WiFi *****/
const char *ssid = "S24";
const char *wifiPassword = "12345678";

/***** MQTT *****/
char *mqttServer = "broker.emqx.io";
int mqttPort = 1883;
const char *mqttUser     = "kstro";
const char *mqttPassword = "2002";
const char *TOPIC_JSON      = "smartank/estado";
const char *TOPIC_CONTROL   = "smartank/control";

/***** Network clients *****/
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

/***** Reles se activa en LOW *****/
const int RELE_ON  = LOW;
const int RELE_OFF = HIGH;

/***** Parametros fisicos o de operacion *****/
float velocidadSonido = 0.0343 / 2.0;
float alturaMaxima = 12.0;

/***** Calibracion PH/TDS *****/
float desplazamientoPh = 2.23;
float pendientePh = -0.18;
float factorTds = 0.5;

/***** Umbrales de tolerancias *****/
float umbralLlenado = 75.0;
const float porcentajeTolerancia = 10.0;
const int lecturasEstableRequeridas = 5;
int contadorEstable = 0;

/***** TDS para auto-drenaje *****/
const float umbralDrenajeTdsPpm = 300.0;

/***** Umbral de seguridad para detener drenaje % *****/
const float UMBRAL_DRENAJE_NIVEL = 10.0; 

/***** Valores por defecto en caso de fallos *****/
const float DISTANCIA_DEFAULT = -1.0;
const float NIVEL_DEFAULT     = 0.0;
const float PH_DEFAULT        = 7.00;
const float TDS_DEFAULT       = 0.0;

/***** Modos y estados *****/
bool modoManual = true;
bool estadoManualLlenado = false;
bool estadoManualDrenaje = false;
bool modoDrenajeTds = false;

/***** Sensor Ultrasonido *****/
#define ULTRA_MUESTRAS 5
const long ULTIMA_LECTURA_MAX_ANTIGUA = 5000;
float ultimaDistanciaValida = DISTANCIA_DEFAULT;
long ultimoTiempoValido = 0;

/***** Estado para publicacion en MQTT *****/
float porcentajeMostrado = 0.0;
long ultimoMillisWifi = 0;
long ultimoPrintMillis = 0;

// Lee el sensor ultrasonico, aplica filtro por mediana y devuelve distancia; 
// Importante: usa cache si no hay lecturas recientes.
float leerUltrasonico() {
  long tiempos[ULTRA_MUESTRAS];
  int contValidas = 0;
  
  // Repetir varias mediciones del sensor ultrasonico y almacenar solo las duraciones validas
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

  long hora_actual = millis();

  // Si no hay lecturas validas pero existe una reciente en cache la retorna
  if (contValidas == 0) {
    if (ultimaDistanciaValida >= 0 && (hora_actual - ultimoTiempoValido) <= 5000) {
      return ultimaDistanciaValida;
    } else {
      return -1.0;
    }
  }

  // Ordenar las duraciones validas para poder obtener la mediana de forma correcta
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
  ultimoTiempoValido = hora_actual;

  return distancia;
}

// Convierte un voltaje leido del sensor TDS a ppm usando la formula.
float convertirVoltajeATds(float v) {
  return (133.42 * v*v*v - 255.86 * v*v + 857.39 * v) * factorTds;
}

// Lee varias muestras del pin PH, promedia y aplica calibracion sencilla para devolver valor pH.
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

// Lee varias muestras del pin TDS, promedia y convierte a ppm.
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

// Enciende el led correspondiente: verde si alto, amarillo si medio, rojo si bajo.
void actualizarIndicadores(float porcentaje) {
  // Si el porcentaje es alto, encender verde mayor a 70%
  if (porcentaje >= 70.0) {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  // Si el porcentaje es intermedio, encender amarillo entre 31% y 69%
  else if (porcentaje > 30.0) {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, HIGH);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  // Si el porcentaje es bajo, encender rojo
  else {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AMARILLO, LOW);
    digitalWrite(PIN_LED_ROJO, HIGH);
  }
}

// Clasifica calidad de agua segun valor TDS y devuelve etiqueta.
String clasificarTds(float tds) {
  if (tds <= 170.0) return "muy buena";
  if (tds <= 300.0) return "buena";
  if (tds <= 375.0) return "regular";
  if (tds <= 500.0) return "mala";
  return "muy mala";
}

// Publica por MQTT el estado completo del sistema: nivel, pH, TDS, estado de bombas y modo actual.
void publicarEstado() {
  // Leer distancia del ultrasonido y convertirla en altura util dentro del tanque
  float distancia = leerUltrasonico();
  float altura = (distancia < 0) ? 0 : alturaMaxima - distancia;
  if (altura < 0) altura = 0;
  if (altura > alturaMaxima) altura = alturaMaxima;
  float porcentajeMedido = (altura / alturaMaxima) * 100.0;

  // Ajustar porcentaje mostrado para compensar error visual o de medicion
  porcentajeMostrado = porcentajeMedido;
  if (porcentajeMedido >= 50.0) {
    porcentajeMostrado = porcentajeMedido + 7.0;
    if (porcentajeMostrado > 100.0) porcentajeMostrado = 100.0;
  }

  // Leer pH y validar si la lectura es valida, si no usar valor por defecto
  float ph = leerPh();
  if (isnan(ph) || ph < 0 || ph > 14) ph = PH_DEFAULT;

  // Leer TDS y validar si es correcto, si no usar valor por defecto
  float tds = leerTds();
  if (isnan(tds) || tds < 0 || tds > 2000) tds = TDS_DEFAULT;

  // Obtener el estado actual de ambas bombas para enviarlo por MQTT
  bool bombaL = (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON);
  bool bombaD = (digitalRead(PIN_BOMBA_DRENAJE) == RELE_ON);

  // Clasificar el TDS en etiquetas de calidad
  String tdsCal = clasificarTds(tds);

  // Construir el JSON que se enviara por MQTT con todas las mediciones y estados
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

  // Enviar el JSON al broker MQTT en el topico asignado
  mqttClient.publish(TOPIC_JSON, json.c_str());
}

// Cambia entre modo manual y automatico 
// al activarse el modo manual limpia todos los flags y apaga bombas para evitar estados anteriores.
void establecerModoManual(bool estadoActual) {
  modoManual = estadoActual;
  Serial.print("--- >>> Modo cambiado a: ");
  Serial.print(modoManual ? "MANUAL" : "AUTOMATICO");
  Serial.println(" ---");

  // Si se activa modo manual, reiniciar estados y asegurar que ambas bombas queden apagadas
  if (modoManual) {
    estadoManualLlenado = false;
    estadoManualDrenaje = false;
    digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
    modoDrenajeTds = false;
    contadorEstable = 0;
  } else {
    // Si se vuelve al modo automatico, reiniciar flags manuales y el contador de estabilidad
    estadoManualLlenado = false;
    estadoManualDrenaje = false;
    contadorEstable = 0;
  }

  // Enviar actualizacion del estado por MQTT si hay conexion
  if (mqttClient.connected()) publicarEstado();
}

// Enciende o apaga el llenado manual; si se desactiva, garantiza que la bomba se apaga de inmediato.
void establecerEstadoManualLlenado(bool estadoActual) {
  estadoManualLlenado = estadoActual;
  Serial.print("--- >>> Llenado manual (flag) set a: ");
  Serial.print(estadoActual ? "ON" : "OFF");
  Serial.println(" ---");

  // Si se desactiva el llenado manual, la bomba debe apagarse inmediatamente
  if (!estadoManualLlenado) {
    digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  }

  // Publicar estado actualizado si hay conexion MQTT
  if (mqttClient.connected()) publicarEstado();
}

// Enciende o apaga manualmente la bomba de drenaje segun el flag recibido.
void establecerEstadoManualDrenaje(bool estadoActual) {
  estadoManualDrenaje = estadoActual;
  Serial.print("--- >>> Drenaje manual (flag) set a: ");
  Serial.print(estadoActual ? "ON" : "OFF");
  Serial.println(" ---");

  // Si esta activo, encender bomba; si no, apagarla inmediatamente
  if (estadoManualDrenaje) {
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
  } else {
    digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
  }

  // Enviar actualizacion del estado por MQTT si se encuentra conectado
  if (mqttClient.connected()) publicarEstado();
}
// Procesa los mensajes que llegan por MQTT en el topico de control; interpreta comandos y ejecuta acciones como cambiar modo o activar bombas.
void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  // Convertir el topic recibido a String para compararlo facilmente
  String topic = String(topicChar);

  // Extraer el mensaje completo del payload byte por byte
  String cmd = "";
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();

  // Crear una version en mayusculas del comando para facilitar comparaciones
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  // Mostrar por consola que comando llego por MQTT
  Serial.print("--- MQTT recibido en ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.print(cmdUpper);
  Serial.println(" ---");

  // Verificar que el mensaje pertenece al topico de control antes de interpretar comandos
  if (topic == String(TOPIC_CONTROL)) {

    // Comando para activar modo MANUAL
    if (cmdUpper == "ON" || cmdUpper == "MODE:ON" || cmdUpper == "MODE_ON") {
      establecerModoManual(true);
    }

    // Comando para activar modo AUTOMATICO
    else if (cmdUpper == "OFF" || cmdUpper == "MODE:OFF" || cmdUpper == "MODE_OFF") {
      establecerModoManual(false);
    }

    // Comando para activar la bomba de llenado manualmente
    else if (cmdUpper == "PUMP_ON" || cmdUpper == "PUMP:ON" || cmdUpper == "LLENADO_ON") {

      // Si el sistema esta en automatico y llega una orden directa de bomba, forzar el cambio a modo manual
      if (!modoManual) {
        Serial.println("--- PUMP_ON recibido: forzando MODO_MANUAL ---");
        establecerModoManual(true);
      }

      // Activar llenado manual
      estadoManualLlenado = true;
      Serial.println("--- EST_MANUAL_LLENADO = true (orden PUMP_ON). ---");

      // Enviar actualizacion del estado si hay conexion MQTT
      if (mqttClient.connected()) publicarEstado();
    }

    // Comando para apagar la bomba de llenado en modo manual
    else if (cmdUpper == "PUMP_OFF" || cmdUpper == "PUMP:OFF" || cmdUpper == "LLENADO_OFF") {
      estadoManualLlenado = false;
      digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
      Serial.println("--- PUMP_OFF recibido: bomba apagada (manual). ---");

      if (mqttClient.connected()) publicarEstado();
    }

    // Comando para activar la bomba de drenaje manualmente
    else if (cmdUpper == "DRENAR_ON" || cmdUpper == "DREN:ON") {

      // Si llega una orden directa de drenaje mientras esta en automatico, cambiar a manual
      if (!modoManual) {
        Serial.println("--- DRENAR_ON recibido: forzando MODO_MANUAL ---");
        establecerModoManual(true);
      }

      // Activar drenaje manual
      estadoManualDrenaje = true;
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
      Serial.println("--- DRENAR_ON: drenaje activado (manual). ---");

      if (mqttClient.connected()) publicarEstado();
    }
    // Comando para apagar la bomba de drenaje en modo manual
    else if (cmdUpper == "DRENAR_OFF" || cmdUpper == "DREN:OFF") {
      estadoManualDrenaje = false;
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
      Serial.println("--- DRENAR_OFF: drenaje apagado (manual). ---");
      if (mqttClient.connected()) publicarEstado();
    }
    else {
      Serial.println("--- Comando no reconocido. ---");
    }
  }
}

// Reintenta conectar cliente MQTT hasta que la conexion se establezca.
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("--- Conectando MQTT (EMQX)... ---");
    if (mqttClient.connect("ESP32_SMARTANK", mqttUser, mqttPassword)) {
      Serial.println("--- MQTT Conectado! ---");
      mqttClient.subscribe(TOPIC_CONTROL);
      publicarEstado();
    } else {
      Serial.print("--- Error MQTT: ");
      Serial.print(mqttClient.state());
      Serial.println(" ---");
      delay(3000);
    }
  }
}

// Imprime el estado de la conexion WiFi cada 30 segundos 
void printWifiStatus() {
  long hora_actual = millis();
  // Si han pasado 30000 ms desde la ultima impresion del estado wifi, imprimir
  if (hora_actual - ultimoMillisWifi >= 30000) {
    Serial.println("\n--- ---------------------------------------- ---");
    Serial.println("--- Estado WIFI: ---");
    Serial.print("--- IP: ");
    Serial.print(WiFi.localIP());
    Serial.println(" ---");
    Serial.println("--- ---------------------------------------- ---");
    ultimoMillisWifi = hora_actual;
  }
}

// Espera el tiempo indicado manteniendo activo el loop MQTT y reconectando si es necesario.
void waitWithMqtt(long ms) {
  long inicio = millis();
  // Mientras no se cumpla el tiempo, mantener el loop de MQTT
  while (millis() - inicio < ms) {
    if (!mqttClient.connected()) reconnect();
    mqttClient.loop();
    delay(50);
  }
}

// Ejecuta el ciclo de drenaje 5000ms y llenado 4500ms para TDS 
// durante drenaje comprueba nivel y corta si baja al umbral.
void ejecutarCicloDrenajeTds() {
  if (modoManual) return;

  Serial.println("--- >>> Iniciando ciclo TDS: drenaje 5s -> llenado 4.5s ---");

  // asegurar que la bomba de llenado este apagada antes de empezar a drenar
  digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  delay(20);

  // activar drenaje
  digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
  if (mqttClient.connected()) publicarEstado();

  long inicio = millis();
  // Durante 5 segundos drenar, pero comprobar nivel periodicamente para cortar si baja demasiado
  while (millis() - inicio < 5000 && !modoManual) {
    // leer nivel actual y calcular porcentaje
    float distanciaAct = leerUltrasonico();
    float alturaAct = (distanciaAct < 0) ? 0 : alturaMaxima - distanciaAct;
    if (alturaAct < 0) alturaAct = 0;
    if (alturaAct > alturaMaxima) alturaAct = alturaMaxima;
    float porcentajeAct = (alturaAct / alturaMaxima) * 100.0;

    // Si el nivel esta por debajo o igual al umbral de seguridad, cortar drenaje
    if (porcentajeAct <= UMBRAL_DRENAJE_NIVEL) {
      Serial.println("--- >>> Nivel <= UMBRAL_DRENAJE_NIVEL durante drenaje TDS -> apagando bomba de drenaje (seguridad). ---");
      break;
    }

    if (!mqttClient.connected()) reconnect();
    mqttClient.loop();
    delay(50);
  }

  // apagar drenaje 
  digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
  if (mqttClient.connected()) publicarEstado();
  delay(150);

  // llenar durante 4.5s
  digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
  if (mqttClient.connected()) publicarEstado();
  waitWithMqtt(4500);
  digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
  if (mqttClient.connected()) publicarEstado();

  Serial.println("--- >>> Fin de ciclo TDS evaluamos el TDS nuevamente ---");
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
  Serial.print("--- Conectando a WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println(" ---");
  Serial.println("--- WiFi conectado! ---");

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  reconnect();

  ultimoPrintMillis = millis();
}

void loop() {
  // Imprime periodicamente informacion del WiFi si corresponde
  printWifiStatus();

  // Asegurar conexion MQTT y procesar mensajes entrantes
  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();

  // Leer distancia del sensor ultrasonico y preparar variables de nivel
  float distancia = leerUltrasonico();
  float porcentajeMedido = 0.0;
  float altura = 0.0;

  // Si la lectura es invalida, usar valores por defecto para evitar acciones peligrosas
  if (distancia < 0) {
    distancia = DISTANCIA_DEFAULT;
    porcentajeMedido = NIVEL_DEFAULT;
    altura = 0.0;
  } else {
    // Calcular altura util dentro del tanque a partir de la distancia medida
    altura = alturaMaxima - distancia;
    if (altura < 0) altura = 0;
    if (altura > alturaMaxima) altura = alturaMaxima;
    // Convertir altura a porcentaje del tanque
    porcentajeMedido = (altura / alturaMaxima) * 100.0;
  }

  // Leer pH y validar que este dentro de rango; si no, usar valor por defecto
  float ph = leerPh();
  if (isnan(ph) || ph < 0 || ph > 14) ph = PH_DEFAULT;

  // Leer TDS y validar que este en rango razonable; si no, usar valor por defecto
  float tds = leerTds();
  if (isnan(tds) || tds < 0 || tds > 2000) tds = TDS_DEFAULT;

  // Actualizar LEDs indicadores segun el porcentaje medido
  actualizarIndicadores(porcentajeMedido);

  // Obtener tiempo actual para temporizaciones y salidas por consola
  long hora_actual = millis();

  // Si estamos en modo AUTOMATICO, gestionar logica automatica (TDS y llenado)
  if (!modoManual) {
    // Si el TDS esta por encima del umbral y la distancia es valida -> entrar en modo drenaje TDS
    if (tds > umbralDrenajeTdsPpm && distancia >= 0) {
      modoDrenajeTds = true;
      Serial.println("--- >>> TDS alto detectado en automatico -> entrando en MODO_DRENAGE_TDS. ---");
      // Bucle que repite ciclos de drenaje/llenado hasta que TDS baje o se cambie el modo
      while (!modoManual) {
        // Leer TDS actual y proteger contra lecturas NaN usando el ultimo valor conocido
        float tdsAhora = leerTds();
        if (isnan(tdsAhora)) tdsAhora = tds;
        Serial.print("---    > TDS actual (antes de ciclo): ");
        Serial.print(tdsAhora);
        Serial.println(" ---");
        // Si el TDS ya esta por debajo o igual al umbral, salir del modo drenaje
        if (tdsAhora <= umbralDrenajeTdsPpm) {
          Serial.println("---    > TDS ya bajo o igual al umbral; saliendo del modo drenaje TDS. ---");
          modoDrenajeTds = false;
          break;
        }
        // Ejecutar un ciclo de drenaje/llenado especifico para reducir TDS
        ejecutarCicloDrenajeTds();
        // Pequeña espera manteniendo MQTT activo antes de volver a evaluar TDS
        waitWithMqtt(300); // espera pequena entre ciclos para estabilizar
        // Evaluar TDS despues del ciclo para decidir si repetir
        float tdsAfter = leerTds();
        Serial.print("---    > TDS despues del ciclo: ");
        Serial.print(tdsAfter);
        Serial.println(" ---");
        // Si el TDS quedo dentro del rango, salir del modo drenaje; si no, repetir
        if (tdsAfter <= umbralDrenajeTdsPpm) {
          Serial.println("---    > TDS corregido. Finalizando MODO_DRENAGE_TDS. ---");
          modoDrenajeTds = false;
          break;
        } else {
          Serial.println("---    > TDS aun alto -> repetir ciclo (siempre en automatico). ---");
        }
      }
    }

    // Si no estamos en modo drenaje por TDS, gestionar llenado automatico y seguridad del drenaje
    if (!modoDrenajeTds) {
      if (distancia >= 0) {
        // Calcular umbrales de arranque y parada para la bomba de llenado
        float onThreshold = umbralLlenado - porcentajeTolerancia;
        float offThreshold = umbralLlenado;
        // Si el nivel esta por debajo del onThreshold, incrementar contador de estabilidad
        if (porcentajeMedido < onThreshold) {
          contadorEstable++;
          // Si el nivel se ha mantenido bajo durante suficientes lecturas, activar bomba de llenado
          if (contadorEstable >= lecturasEstableRequeridas) {
            if (digitalRead(PIN_BOMBA_LLENADO) != RELE_ON) {
              digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
              Serial.println("--- AUTO: Arrancando bomba (condicion estable por debajo del onThreshold). ---");
            }
            // Evitar overflow del contador manteniendolo en el maximo necesario
            contadorEstable = lecturasEstableRequeridas;
          }
        } else {
          // Si la condicion de bajo nivel no se cumple, resetear el contador de estabilidad
          contadorEstable = 0;
        }
        // Si el nivel alcanza el offThreshold, apagar la bomba de llenado (condicion de parada)
        if (porcentajeMedido >= offThreshold) {
          if (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON) {
            digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
            Serial.println("--- AUTO: Apagando bomba (porcentaje_medido >= offThreshold). ---");
          }
          contadorEstable = 0;
        }
      } else {
        // Si no hay distancia valida, asegurar que la bomba de llenado este apagada por seguridad
        digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
        contadorEstable = 0;
      }

      // Seguridad adicional: si la bomba de drenaje esta encendida en automatico y el nivel baja al umbral de seguridad, apagarla
      if (digitalRead(PIN_BOMBA_DRENAJE) == RELE_ON) {
        if (porcentajeMedido <= UMBRAL_DRENAJE_NIVEL) {
          digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
          modoDrenajeTds = false; // asegurar que no permanezcamos en modo drenaje
          Serial.println("--- AUTO: Bomba de drenaje apagada porque nivel <= UMBRAL_DRENAJE_NIVEL (seguridad). ---");
          if (mqttClient.connected()) publicarEstado();
        }
      } else {
        // Si la bomba de drenaje no esta activa, garantizar salida en OFF
        digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
      }
    }
  } else {
    // MODO MANUAL
    // Control manual de llenado: arrancar la bomba si el usuario lo solicito y apagar automaticamente al alcanzar umbral
    if (estadoManualLlenado) {
      if (porcentajeMedido >= umbralLlenado) {
        digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
        estadoManualLlenado = false;
        Serial.println("--- Bomba apagada automaticamente por alcanzar umbral (modo manual). ---");
        if (mqttClient.connected()) publicarEstado();
      } else {
        digitalWrite(PIN_BOMBA_LLENADO, RELE_ON);
      }
    } else {
      // Si no esta activado el llenado manual, asegurar que la bomba este apagada
      digitalWrite(PIN_BOMBA_LLENADO, RELE_OFF);
    }

    // Control manual de drenaje con seguridad: si el usuario activo drenaje, 
    // se permite salvo que el nivel baje al umbral de seguridad
    if (estadoManualDrenaje) {
      if (porcentajeMedido <= UMBRAL_DRENAJE_NIVEL) {
        // Apagar drenaje por seguridad aunque el modo sea manual
        digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
        estadoManualDrenaje = false;
        Serial.println("--- MANUAL: Bomba de drenaje apagada automaticamente porque nivel es menor al umbral de drenaje ---");
        if (mqttClient.connected()) publicarEstado();
      } else {
        // Mantener drenaje activo mientras el nivel este por encima del umbral de seguridad
        digitalWrite(PIN_BOMBA_DRENAJE, RELE_ON);
      }
    } else {
      // Si no esta activado drenaje manual, asegurar que la bomba este apagada
      digitalWrite(PIN_BOMBA_DRENAJE, RELE_OFF);
    }
  }

  // Preparar estados y etiquetas para publicar por MQTT
  bool bombaLEstado = (digitalRead(PIN_BOMBA_LLENADO) == RELE_ON);
  bool bombaDEstado = (digitalRead(PIN_BOMBA_DRENAJE) == RELE_ON);

  String tdsCal = clasificarTds(tds);

  // Ajuste visual del porcentaje que se mostrara por MQTT (debido a fallos del ultrasonico en cortas distancias) 
  porcentajeMostrado = porcentajeMedido;
  if (porcentajeMedido >= 55.0) {
    porcentajeMostrado = porcentajeMedido + 7.5;
    if (porcentajeMostrado > 100.0) porcentajeMostrado = 100.0;
  }

  // Construir JSON con estado completo para publicar por MQTT
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

  // Publicar el JSON si hay conexion MQTT
  if (mqttClient.connected()) mqttClient.publish(TOPIC_JSON, json.c_str());

  // Imprimir estado en consola cada 5 segundos para monitorear el sistema
  if (hora_actual - ultimoPrintMillis >= 5000) {
    ultimoPrintMillis = hora_actual;
    Serial.println("--- ==== MOSTRANDO ESTADO DEL SISTEMA ==== ---");
    Serial.print("--- Modo actual: ");
    Serial.print(modoManual ? "MANUAL" : "AUTOMATICO");
    Serial.println(" ---");
    Serial.print("--- Bomba llenado: ");
    Serial.print(bombaLEstado ? "ENCENDIDA" : "APAGADA");
    Serial.println(" ---");
    Serial.print("--- Bomba drenaje: ");
    Serial.print(bombaDEstado ? "ENCENDIDA" : "APAGADA");
    Serial.println(" ---");
    Serial.print("--- Distancia (cm): ");
    Serial.print(distancia, 2);
    Serial.println(" ---");
    Serial.print("--- Porcentaje medido: ");
    Serial.print(porcentajeMedido, 1);
    Serial.println(" ---");
    Serial.print("--- Porcentaje mostrado (MQTT): ");
    Serial.print(porcentajeMostrado, 1);
    Serial.println(" ---");
    Serial.print("--- TDS (ppm): ");
    Serial.print(tds);
    Serial.println(" ---");
    Serial.print("--- TDS calidad: ");
    Serial.print(tdsCal);
    Serial.println(" ---");
    Serial.println("--- JSON enviado: ---");
    Serial.println(json);
    Serial.println("--- ======================================= ---");
  }

  // Delay para no sobresaturar la placa 
  delay(200);
}
