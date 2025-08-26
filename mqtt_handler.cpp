/*
  Project:      benchFace
  Description:  MQTT functions for benchFace
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// required external functions and data structures
extern void debugMessage(String messageText, uint8_t messageLevel);
extern const char* generateMQTTTopic(String key);
extern bool faceSeen;

// MQTT setup
#include <PubSubClient.h>
extern PubSubClient mqtt;

void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
  debugMessage(String("new topic from broker ") + topic, 1);
  const char* targetTopic = generateMQTTTopic(VALUE_KEY_LIGHT);
  if (strcmp(targetTopic, topic) == 0) {
    String message;
    message.reserve(length);
    for (unsigned int loop = 0; loop < length; loop++) message += (char)payload[loop];
    uint8_t status = message.toInt();
    debugMessage(String("MQTT; change light to ") + (status == 1 ? "On" : "Off"),1);
    digitalWrite(hardwareRelayPin, status);
    if (!status)
      faceSeen = false; // allows for a person to be detected after a remote override of light state
  }
}

bool mqttConnect() {
  if (mqttBrokerConfig.host.isEmpty() || mqttBrokerConfig.port == 0)
    return false;

  mqtt.setServer(mqttBrokerConfig.host.c_str(), mqttBrokerConfig.port);
  // Serial.printf("Connecting to MQTT %s:%u ...\n", mqttBrokerConfig.host.c_str(), mqttBrokerConfig.port);

  bool connected;
  if (mqttBrokerConfig.user.length() > 0) {
    connected = mqtt.connect(endpointPath.deviceID.c_str(), mqttBrokerConfig.user.c_str(), mqttBrokerConfig.password.c_str());
  } else {
    connected = mqtt.connect(endpointPath.deviceID.c_str());
  }

  if (connected) {
    debugMessage("MQTT connected",1);
  } else {
    Serial.printf("MQTT connect failed, rc=%d\n", mqtt.state());
  }
  return connected;
}

void mqttPublish(const char* topic, const String& payload) {
  if (!mqtt.connected()) return;
  mqtt.publish(topic, payload.c_str());
  // Serial.printf("[MQTT OUT] %s => %s\n", topic, payload.c_str());
}

void mqttSubscribe(const char* topic) {
  if (!mqtt.connected()) return;
  mqtt.subscribe(topic);
  Serial.printf("[MQTT SUB] %s\n", topic);
}