// --- Sensor TDS con ESP8266 ---
// Mide los sólidos disueltos totales (ppm) en el agua

#define PIN_TDS A0      // Pin analógico del sensor
#define VOLTAJE_REF 3.3 // Voltaje de referencia del ESP8266

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TDS, INPUT);
  Serial.println("Iniciando medición de TDS...");
}

void loop() {
  // Leer valor analógico del sensor (0–1023)
  int lectura = analogRead(PIN_TDS);

  // Convertir la lectura a voltaje
  float voltaje = lectura * (VOLTAJE_REF / 1024.0);

  // Calcular TDS (ppm) con fórmula de conversión aproximada
  float tds = (133.42 * pow(voltaje, 3)
              - 255.86 * pow(voltaje, 2)
              + 857.39 * voltaje) * 0.5;

  // Mostrar resultado en el monitor serial
  Serial.print("Voltaje: ");
  Serial.print(voltaje, 2);
  Serial.print(" V\tTDS: ");
  Serial.print(tds, 0);
  Serial.println(" ppm");

  delay(1000); // Esperar 1 segundo antes de la siguiente lectura
}
