Smartank – Sistema Inteligente de Monitoreo y Control de Cisterna (ESP32 + MQTT)

Smartank es un sistema IoT para monitorear y controlar un tanque de agua usando ESP32, sensores de ultrasonido, pH y TDS, y comunicación MQTT.

Contenido:
- Monitoreo de nivel, pH y TDS
- Control automático y manual de bombas
- Dashboard web conectado por MQTT
- Modo auto-drenaje por TDS alto
- Instrucciones de instalación y uso

Requisitos:
- ESP32 Dev Module
- Sensor Ultrasonico HC-SR04
- Sensor pH
- Sensor TDS
- Relés activos en LOW
- Broker MQTT (ej: EMQX)

Instalación:
1. Cargar Smartank.ino en el ESP32.
2. Editar credenciales WiFi y MQTT.
3. Abrir el dashboard web y conectar al broker MQTT.
