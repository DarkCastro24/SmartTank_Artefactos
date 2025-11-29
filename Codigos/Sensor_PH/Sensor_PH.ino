/*
 * Prueba básica de sensor de pH con ESP32
 * Lee el valor analógico y muestra el valor estimado de pH en el monitor serie
*/

// Pin analógico del sensor de PH
const int phPin = 34;  
float calibration = 21.34; 

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando lectura de sensor de pH...");
}

void loop() {
  int analogValue = analogRead(phPin);
  
  // Convertir lectura ADC 
  float voltage = analogValue * (3.3 / 4095.0);
  
  // Fórmula de estimación básica 
  float phValue = 7 + ((2.5 - voltage) / 0.18);

  Serial.print("ADC: ");
  Serial.print(analogValue);
  Serial.print(" | Voltaje: ");
  Serial.print(voltage, 2);
  Serial.print(" V | pH estimado: ");
  Serial.println(phValue, 2);
  
  delay(1000); 
}
