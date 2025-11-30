#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"   //********* Librería para el sensor DHT11 *********

//********DELAY PARA MOSTRAR ESTADO DEL WIFI*********
unsigned long interval = 30000; // delay en ms
unsigned long previousMillis = 0;

WiFiClient wlanclient;
PubSubClient mqttClient(wlanclient);

//***************CONFIGURACIÓN DE RED****************
const char *ssid = "**************";
const char *passwd = "************";

//***************CONFIGURACIÓN DE MQTT***************
char *server = "*************";
int port = 1883;

const char *mqtt_user = "*********";
const char *mqtt_password = "*****";

// Definir todos los "topics" a los cuales se desea suscribir
const char* topics[] = {
  "/test/message",
  "/sensor/temperature",  // topic para temperatura
  "/sensor/humidity"      // topic para humedad
};

//***************CONFIGURACIÓN DEL SENSOR DHT11***************
#define DHTPIN D4          // Pin digital donde se conecta el DHT11
#define DHTTYPE DHT11      // Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);  // Crear objeto DHT

//***************CALLBACK DE RESOLUCIÓN A SUBSCRIBES***************
void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  Serial.println();

  //*********Obtener el topic convertido a tipo String*************
  String topic = String(topicChar);
  Serial.print("Message arrived on Topic: ");
  Serial.println(topic);

  //*********Obtener el mensaje convertido a tipo String***********
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];  // Añadir cada carácter al String
  }

  //**********Definir manejo de mensajes entrantes por topic*******
  //Ej:
  if (topic == "/test/message") {
    Serial.println(message);
  }
}

//***************FUNCIONES PARA PUBLICAR Y SUSCRIBIR***************
boolean publishToTopic(char *topic, char *message) {
  return mqttClient.publish(topic, message);
}

void subscribeToTopics() {
  const int numTopics = sizeof(topics) / sizeof(topics[0]);

   for (int i = 0; i < numTopics; i++) {
      mqttClient.subscribe(topics[i]);
  }
}

//***************DEFINICIÓN DE VARIABLES GLOBALES******************
//Ej. int LED = D2;

void setup() {
  //*********Sección para inicializar sensores*********
  Serial.begin(115200);
  dht.begin(); // Inicializar el sensor DHT11

  //*********Configurar conexión wifi*************
  WiFi.begin(ssid, passwd);

  Serial.print("Connecting to AP");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected to WiFi Access Point, Got an IP address: ");
  Serial.println(WiFi.localIP());

  // Configurar la reconexión automática ante perdida de señal
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  //*********Configurar conexión MQTT*************
  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqttCallback);

  // Realizar conexión al MQTT Broker
  if (mqttClient.connect("ESP-DHT11", mqtt_user, mqtt_password)) {
    Serial.println("Connected to MQTT Broker");
  } else {
    Serial.println("MQTT Broker connection failed");
    Serial.println(mqttClient.state());
    delay(200);
  }

  // suscribirse a los temas.
  subscribeToTopics();
}

void loop() {
  // Imprime la señal de la red cada <interval> segundos
  printWifiStatus();
  
  // verificar si no hay conexión con el broker, si es así reconectarse:
  if (!mqttClient.connected()) {
    reconnect(); 
  }

  // Loop de funcionamiento de la libreria PubSubClient
  mqttClient.loop();

  //********Lectura del sensor DHT11 y envío por MQTT********
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error al leer el sensor DHT11");
    delay(2000);
    return;
  }

  // Convertir los valores a char* usando funciones auxiliares
  char *tempString = floatToChar(temperature, 2);
  char *humString = floatToChar(humidity, 2);

  // Publicar los datos en los topics correspondientes
  publishToTopic("/sensor/temperature", tempString);
  publishToTopic("/sensor/humidity", humString);

  // Mostrar datos por monitor serie
  Serial.print("Temperatura: ");
  Serial.print(tempString);
  Serial.print(" °C | Humedad: ");
  Serial.print(humString);
  Serial.println(" %");

  // Liberar memoria dinámica
  free(tempString);
  free(humString);

  // Esperar 5 segundos antes de la siguiente lectura
  delay(5000);
}

//***************FUNCIÓN DE RECONEXIÓN MQTT***************
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println();
    Serial.println("Trying to connect to the MQTT broker...");

    if (mqttClient.connect("ESP-DHT11", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");

      // suscribirse nuevamente a los temas si la conexión regresa.
      subscribeToTopics();
    } else {
      Serial.print("Fallo, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Trying to connect each 5 seconds");
      // Esperar 5 segundos antes de reintentar
      delay(5000);
    }
    printWifiStatus();
  }
}

void printWifiStatus() {
  // Imprime la señal de la red cada <interval> segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    Serial.println("");
    Serial.println("----------------------------------------");
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("Wifi Configured SSID cannot be reached");
        break;
      case WL_CONNECTED:
        Serial.println("Connection Wifi successfully established");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("Wifi Connection failed");
        break;
    }
    Serial.printf("Connection status: %d\n", WiFi.status());
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    Serial.println("----------------------------------------");
    previousMillis = currentMillis;
  }
}

//***************FUNCIONES AUXILIARES DE CONVERSIÓN***************

//***************float -> *char********************
char* floatToChar(float number, int precision) {
    // Calcular el tamaño necesario (incluyendo el signo y el punto decimal)
    int length = snprintf(NULL, 0, "%.*f", precision, number);
    
    // Asignar memoria para la cadena resultante
    char* result = (char*)malloc(length + 1); // +1 para el terminador nulo
    
    // Verificar si la asignación fue exitosa
    if (result != NULL) {
        snprintf(result, length + 1, "%.*f", precision, number);
    }
    
    return result;
}

//***************int -> *char********************
char* intToChar(int number) {
    // Calcular el tamaño necesario (incluyendo el signo)
    int length = snprintf(NULL, 0, "%d", number);
    
    // Asignar memoria para la cadena resultante
    char* result = (char*)malloc(length + 1); // +1 para el terminador nulo
    
    // Verificar si la asignación fue exitosa
    if (result != NULL) {
        snprintf(result, length + 1, "%d", number);
    }
    
    return result;
}
