# Smartank – Sistema Inteligente de Monitoreo y Control de Cisterna (ESP32 + MQTT)

Smartank es un sistema IoT diseñado para monitorear y controlar un tanque de agua en tiempo real utilizando un **ESP32**, sensores de **ultrasonido**, **pH**, **TDS**, y comunicación **MQTT**.  
Permite visualizar los datos desde un panel web y controlar las bombas en modo **automático** o **manual**.

---

## Contenido del Sistema
- Monitoreo de nivel del agua (cm / %)
- Lectura de pH y TDS
- Control automático y manual de bombas mediante relés
- Dashboard web conectado por MQTT (WebSockets)
- Modo automático de **drenaje por TDS alto**
- Pasos de instalación y configuración

---

## Requisitos de Hardware
- **ESP32 Dev Module**
- Sensor **Ultrasonido HC-SR04**
- Sensor **pH**
- Sensor **TDS**
- Módulo de **Relés (activos en LOW)**
- Broker **MQTT Mosquitto**

---

## Instalación
1. Cargar **Smartank.ino** en el ESP32 usando Arduino IDE o PlatformIO.
2. Configurar credenciales **WiFi** y **MQTT** en el código fuente.
3. Abrir el panel web y conectar al broker MQTT para visualizar los datos.

---

