#define SensorPin A0
float calibration_value = 21.34; // Ajusta durante calibración
int sensorValue = 0;
float voltage, phValue;

void setup() {
  Serial.begin(115200);
  pinMode(SensorPin, INPUT);
}

void loop() {
  sensorValue = analogRead(SensorPin);
  voltage = sensorValue * (3.3 / 1023.0); // Convierte a voltaje (3.3 V ADC)
  phValue = 7 + ((2.5 - voltage) / 0.18); // Fórmula aproximada

  Serial.print("Voltaje: ");
  Serial.print(voltage, 2);
  Serial.print(" V  |  pH: ");
  Serial.println(phValue, 2);

  delay(1000);
}
