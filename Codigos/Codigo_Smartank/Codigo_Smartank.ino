/*
  CODIGO SMARTANK   Fecha: 09/11/2025

  CONEXIONES DE LOS ELEMENTOS (SELECCIONAR PLACA ESP32 Dev Module)
  ----------------------------------------------------------
  Sensor ultrasónico (sensor analogico):
    - VCC  → 3.3V 
    - GND  → GND
    - TRIG → GPIO 5
    - ECHO → GPIO 18

  Sensor de pH (sensor analogico):
    - V+   → 3.3V
    - GND  → GND
    - PO   → GPIO 34 (ADC1)

  Sensor TDS (sensor analogico):
    - VCC  → 3.3V
    - GND  → GND
    - AOUT → GPIO 35 (ADC1)

  Bombas de agua 
    - Bomba de drenaje → GPIO 26
    - Bomba de llenado → GPIO 27

  LEDs indicadores:
    - LED Verde    → GPIO 14
    - LED Amarillo → GPIO 12
    - LED Rojo     → GPIO 13
*/

// ---------------------- CONFIGURACION DE PINES -------------------------
#define PIN_TRIG            5
#define PIN_ECHO            18

#define PIN_PH              34
#define PIN_TDS             35

#define PIN_BOMBA_DRENAJE   26
#define PIN_BOMBA_LLENADO   27

#define PIN_LED_VERDE       14
#define PIN_LED_AMARILLO    12
#define PIN_LED_ROJO        13

// ---------------------- PARAMETROS GENERALES ---------------------------

float VELOCIDAD_SONIDO = 0.0343 / 2.0; 
float MAX_ALTURA = 30.0; // Del tanque

// Calibracion pH
float DESPLAZAMIENTO_PH = 0.00;  // offset para pH 7
float PENDIENTE_PH      = -0.18; // voltios por unidad de pH

// Calibracion TDS
float FACTOR_TDS = 0.5; // factor de corrección

// Umbrales TDS (ppm)
int TDS_BAJO = 250;
int TDS_ALTO = 800;

// Umbral de nivel para bomba de llenado (%)
float UMBRAL_LLENADO = 90.0;

// ---------------------- CONTROL MANUAL/AUTOMATICO (Version beta) ----------------------
bool MODO_MANUAL = false; // true = manual, false = automático
bool EST_MANUAL_LLENADO = false; // estado manual bomba llenado
bool EST_MANUAL_DRENAJE = false; // estado manual bomba drenaje

void establecerModoManual(bool habilitado) {
  MODO_MANUAL = habilitado;
  Serial.print("Modo manual: ");
  Serial.println(MODO_MANUAL ? "ACTIVADO" : "DESACTIVADO");
}

void establecerEstadoManualLlenado(bool encender) {
  EST_MANUAL_LLENADO = encender;
  Serial.print("Bomba de llenado (manual): ");
  Serial.println(EST_MANUAL_LLENADO ? "ENCENDIDA" : "APAGADA");
}

void establecerEstadoManualDrenaje(bool encender) {
  EST_MANUAL_DRENAJE = encender;
  Serial.print("Bomba de drenaje (manual): ");
  Serial.println(EST_MANUAL_DRENAJE ? "ENCENDIDA" : "APAGADA");
}

// ---------------------- FUNCIONES DE LECTURA ---------------------------
float leerUltrasonico() {
  // Retorna distancia en cm 
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duracion = pulseIn(PIN_ECHO, HIGH, 30000); // milisegundos
  float distancia = duracion * VELOCIDAD_SONIDO;  // centimetros
  return distancia;
}

float convertirVoltajeATDS(float voltaje) {
  // Retorna TDS estimado en ppm
  float ec = (133.42 * voltaje * voltaje * voltaje
             -255.86 * voltaje * voltaje
             +857.39 * voltaje) * FACTOR_TDS;
  return ec;
}

float leerPH() {
  int muestras = 15;
  unsigned long acumulado = 0;
  for (int i = 0; i < muestras; i++) {
    acumulado += analogRead(PIN_PH);
    delay(10);
  }
  float promedio_adc = acumulado / (float)muestras;
  float voltaje = promedio_adc * (3.3 / 4095.0);          
  float ph = 7.0 + DESPLAZAMIENTO_PH + (2.5 - voltaje) / PENDIENTE_PH;
  return ph;
}

float leerTDS() {
  int muestras = 15;
  unsigned long acumulado = 0;
  for (int i = 0; i < muestras; i++) {
    acumulado += analogRead(PIN_TDS);
    delay(10);
  }
  float promedio_adc = acumulado / (float)muestras;
  float voltaje = promedio_adc * (3.3 / 4095.0); // voltios         
  float tds = convertirVoltajeATDS(voltaje); // ppm
  return tds;
}

// ---------------------- INDICADORES Y ACTUADORES -----------------------
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

// ---------------------- CONFIGURACIÓN INICIAL --------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_BOMBA_DRENAJE, OUTPUT);
  pinMode(PIN_BOMBA_LLENADO, OUTPUT);
  digitalWrite(PIN_BOMBA_DRENAJE, LOW);
  digitalWrite(PIN_BOMBA_LLENADO, LOW);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARILLO, LOW);
  digitalWrite(PIN_LED_ROJO, LOW);

  // ADC ESP32
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PH,  ADC_11db); 
  analogSetPinAttenuation(PIN_TDS, ADC_11db);

  Serial.println("Sistema de monitoreo iniciado.");

  // Por defecto se activa modo automatico
  establecerModoManual(false);
}

// ---------------------- LOOP PRINCIPAL ---------------------------------
void loop() {
  // Nivel de agua: distancia y porcentaje
  float distancia = leerUltrasonico();      
  float altura = MAX_ALTURA - distancia;             
  if (altura < 0) {
    altura = 0;
  }
  if (altura > MAX_ALTURA) {
    altura = MAX_ALTURA;
  }

  float porcentaje = (altura / MAX_ALTURA) * 100.0;  // porcentaje del agua

  Serial.print("Distancia: ");
  Serial.print(distancia, 1);
  Serial.print(" cm | Nivel: ");
  Serial.print(porcentaje, 1);
  Serial.println(" %");

  // Indicadores de nivel
  actualizarIndicadores(porcentaje);

  // Lectura de pH
  float ph = leerPH();
  Serial.print("pH: ");
  Serial.println(ph, 2);

  // Lectura de TDS
  float tds = leerTDS();
  Serial.print("TDS: ");
  Serial.print(tds, 0);
  Serial.println(" ppm");

  if (MODO_MANUAL) {
    // ------ CONTROL MANUAL ------
    digitalWrite(PIN_BOMBA_LLENADO, EST_MANUAL_LLENADO ? HIGH : LOW);
    digitalWrite(PIN_BOMBA_DRENAJE, EST_MANUAL_DRENAJE ? HIGH : LOW);

    Serial.print("Modo manual activo | Llenado: ");
    Serial.print(EST_MANUAL_LLENADO ? "ENCENDIDA" : "APAGADA");
    Serial.print(" | Drenaje: ");
    Serial.println(EST_MANUAL_DRENAJE ? "ENCENDIDA" : "APAGADA");
  } else {
    // ------ CONTROL AUTOMÁTICO ------
    // Bomba de drenaje por TDS
    if (tds > TDS_ALTO) {
      digitalWrite(PIN_BOMBA_DRENAJE, HIGH);
      Serial.println("Bomba de drenaje: ACTIVADA (TDS alto)");
    } else if (tds < TDS_BAJO) {
      digitalWrite(PIN_BOMBA_DRENAJE, LOW);
      Serial.println("Bomba de drenaje: DESACTIVADA (TDS bajo)");
    } else {
      // tds es nulo esto solo ocurre si el sensor esta mal conectado
    }

    // Bomba de llenado por nivel (< 85% del nivel)
    if (porcentaje < UMBRAL_LLENADO) {
      digitalWrite(PIN_BOMBA_LLENADO, HIGH);
      Serial.println("Bomba de llenado: ACTIVADA (nivel bajo)");
    } else {
      digitalWrite(PIN_BOMBA_LLENADO, LOW);
      Serial.println("Bomba de llenado: DESACTIVADA (nivel suficiente)");
    }
  }

  Serial.println("----------------------------------------");
  delay(1000); // ms
}
