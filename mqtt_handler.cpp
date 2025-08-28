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
extern bool faceSeen;

// MQTT setup
#include <PubSubClient.h>
extern PubSubClient mqtt;

const char* generateMQTTTopic(String key)
// Utility function to streamline dynamically generating MQTT topics using site and device 
// parameters defined in config.h and our standard naming scheme using values set in secrets.h
{
  String topic = endpointPath.site + "/" + endpointPath.location + "/" + endpointPath.room +
          "/" + hardwareDeviceType + "/" + endpointPath.deviceID + "/" + key;
  debugMessage(String("Generated MQTT topic: ") + topic,2);
  return(topic.c_str());
}

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
    debugMessage(String("Connected to MQTT broker ") + mqttBrokerConfig.host,1);
  } else {
    debugMessage(String("MQTT connection to ") + mqttBrokerConfig.host + " failed, rc=" + mqtt.state(),1);
  }
  return connected;
}

void mqttPublish(const char* topic, const String& payload) {
  if (!mqtt.connected()) return;
  mqtt.publish(topic, payload.c_str());
  debugMessage(String("MQTT publish topic is ") + topic + ", message is " + payload,2);
}

void mqttSubscribe(const char* topic) {
  if (!mqtt.connected()) return;
  mqtt.subscribe(topic);
  debugMessage(String("Now subscribed to ") + topic,2);
}