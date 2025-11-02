// Avance: Se logro medir el nivel de agua del tanque a traves del sensor ultrasonico       Fecha 01/11/2025
// Ademas se logro encender la bomba mediante el puerto D0

// Declaracion de variables
const int trigPin = D5;   
const int echoPin = D6;  
const int bombaPin = D0;  

// Distancias de calibración en cm (mediciones del sensor ultrasonico)
const float DIST_LLENO = 2.70;
const float DIST_VACIO = 10.22;

long duration;
float distance;
float porcentaje;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(bombaPin, OUTPUT);

  digitalWrite(bombaPin, LOW); // bomba inicialmente apagada
  Serial.println("Sistema de nivel iniciado...");
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;

  // Calcular porcentaje de llenado 
  porcentaje = (DIST_VACIO - distance) / (DIST_VACIO - DIST_LLENO) * 100.0;

  // Limitar valores entre 0% y 100%
  if (porcentaje < 0){
    porcentaje = 0;
  }
  if (porcentaje > 100) {
    porcentaje = 100;
  }

  // Control automático de la bomba 
  if (porcentaje < 95) {
    digitalWrite(bombaPin, HIGH); 
  } else if (porcentaje >= 100) {
    digitalWrite(bombaPin, LOW);  
  }

  // Mostrar la informacion en consola
  Serial.print("Distancia: ");
  Serial.print(distance, 2);
  Serial.print(" cm\tNivel: ");
  Serial.print(porcentaje, 1);
  Serial.println(" %");

  delay(1000); 
}
