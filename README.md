# benchFace
toggle bench light using vision ML or MQTT message

## Purpose
benchLight was designed so that when I sit down at my lab bench, the overhead light immediately turns on. When I walk away, it will turn off after a predefined delay. I'm lazy.

## Associated projects
This project inherited its original code from the Person Sensor sample code and my [status_light_110v](https://github.com/ericklein/status_light_110v) project.

## Features
benchFace uses the Person Sensor look for a face within visual range. If detected, it will send a message to a MQTT broker to indicate that the 110v relay, in this case triggering a light, has been activated. If the Person Sensor doesn't detect a machine within a configurable timeframe, it will send another message to the MQTT broker indicating it has turned off the light.

benchFace subscribes to the same MQTT topic, so it can remotely trigger the same light. This was intentionally done for future Home Assistant integration.

benchFace also logs WiFi RSSI for the device as a separate MQTT topic.

## Target configuration
ESP32 MCU, i2c, and one GPIO pin. 

## Bill of Materials (BOM)
### MCU
- [Adafruit ESP32V2 Feather](https://www.adafruit.com/product/5400)
	- code is failry portable
### WiFi
- all ESP 8266/32 devices is currently supported and tested
### Pinouts
- Person Sensor
    - Stemma QT cable between MCU board and Person Sensor board
        - or connect 3.3v/5v, GND, SDA, SCL on both sides
- IoT Power Relay II -> 110v relay
    - GPIO pin documented in config.h, GND to GND
## Supported network endpoints
### MQTT
- set appropriate parameters in config.h and secrets.h
	- Technical References
		- https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/
## Issues and Feature Requests
- [Github Issues](https://github.com/ericklein/benchFace/issues)

## .plan (big ticket items)
- OTA firmware update support
- transition to individual face recognition
- Home Assistant integration via HassioMQTT

## Supporting Material
- [Useful Objects - Person Sesnor](https://github.com/moonshine-ai/person_sensor_docs?tab=readme-ov-file)
- [Digital Loggers IoT Power Relay II](https://www.digital-loggers.com/iot2.html)