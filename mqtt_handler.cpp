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

// MQTT setup
#include <PubSubClient.h>
extern PubSubClient mqtt;

// void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
//   String msg;
//   msg.reserve(length);
//   for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
//   Serial.printf("[MQTT IN] %s => %s\n", topic, msg.c_str());

//   // Example response:
//   // mqttPublish(mqttBrokerConfig.topicPub.c_str(), String("echo: ") + msg);
// }

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
    // if (mqttBrokerConfig.topicSub.length() > 0) {
    //   mqtt.subscribe(mqttBrokerConfig.topicSub.c_str());
    //   Serial.printf("Subscribed to %s\n", mqttBrokerConfig.topicSub.c_str());
    // }
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

// void mqttSubscribe(const char* topic) {
//   if (!mqtt.connected()) return;
//   mqtt.subscribe(topic);
//   Serial.printf("[MQTT SUB] %s\n", topic);
// }