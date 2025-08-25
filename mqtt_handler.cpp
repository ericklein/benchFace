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

const char* generateMQTTTopic(String key)
// Utility function to streamline dynamically generating MQTT topics using site and device 
// parameters defined in config.h and our standard naming scheme using values set in secrets.h
{
  String topic = endpointPath.site + "/" + endpointPath.location + "/" + endpointPath.room +
          "/" + hardwareDeviceType + "/" + endpointPath.deviceID + "/" + key;
  debugMessage(String("Generated MQTT topic: ") + topic,2);
  return(topic.c_str());
}

// MQTT setup
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
extern Adafruit_MQTT_Client bl_mqtt;
Adafruit_MQTT_Subscribe benchLightSub = Adafruit_MQTT_Subscribe(&bl_mqtt, MQTT_SUB_TOPIC);

bool mqttConnect()
// Connects and reconnects to MQTT broker, call as needed to maintain connection
{
  // exit if already connected
  if (bl_mqtt.connected())
  {
    debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER,2);
    return true;
  }

  // Q: does this need to be signed?
  int8_t mqttErr;

  for(uint8_t loop = 1; loop <= networkConnectAttemptLimit; loop++)
  {
    if ((mqttErr = bl_mqtt.connect()) == 0)
    {
      debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER,1);
      return true;
    }

    // report problem
    bl_mqtt.disconnect();
    debugMessage(String("MQTT connection attempt ") + loop + " of " + networkConnectAttemptLimit + " failed with error msg: " + bl_mqtt.connectErrorString(mqttErr),1);
    delay(timeNetworkConnectTimeoutMS);
  }
  // MQTT connection did not happen after multiple attempts
  return false;
}

bool mqttDeviceWiFiUpdate(uint32_t rssi)
{
  bool published = false;

  if (mqttConnect()){
    const char* topic = generateMQTTTopic(VALUE_KEY_RSSI);  // Generate topic using config.h and secrets.h parameters
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&bl_mqtt, topic);

    if (rssiLevelPub.publish(rssi))
    {
      debugMessage("MQTT publish: WiFi RSSI succeeded",2);
      published = true;
    }
    else
      debugMessage("MQTT publish: WiFi RSSI failed",2);
  }
  else
    debugMessage("No MQTT connection to publish with",1);
  return(published);
}

bool mqttDeviceLightUpdate(bool status)
{
  bool published = false;

  if (mqttConnect()){
    const char* topic = generateMQTTTopic(VALUE_KEY_LIGHT);  // Generate topic using config.h and secrets.h parameters
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish lightPub = Adafruit_MQTT_Publish(&bl_mqtt, topic);

    if (lightPub.publish((uint32_t)status)) {
    debugMessage("MQTT publish: light status succeeded",1);
    published = true;
    }
    else
      debugMessage("MQTT publish: light status failed",1);
  }
  else
    debugMessage("No MQTT connection to publish with",1);
  return(published);
}

uint8_t mqttBenchLightMessage()
{
  debugMessage("Checking mqtt subscription",2);
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = bl_mqtt.readSubscription(mqttSubSampleIntervalMS))) 
  {
    if (subscription == &benchLightSub)
    {
      debugMessage(String("Received '") + ((char *)benchLightSub.lastread) + "' from " + MQTT_SUB_TOPIC,1); 
      return (atol((char *)benchLightSub.lastread) == 1) ? 1 : 0;
    }
  }
  return 2;  // 2 indicates no message received
}

// new MQTT

// void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
//   String msg;
//   msg.reserve(length);
//   for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
//   Serial.printf("[MQTT IN] %s => %s\n", topic, msg.c_str());

//   // Example response:
//   // mqttPublish(cfg.topicPub.c_str(), String("echo: ") + msg);
// }

// bool mqttConnect() {
//   if (cfg.host.isEmpty() || cfg.port == 0) return false;

//   mqtt.setServer(cfg.host.c_str(), cfg.port);
//   Serial.printf("Connecting to MQTT %s:%u ...\n", cfg.host.c_str(), cfg.port);

//   bool ok;
//   if (cfg.user.length() > 0) {
//     ok = mqtt.connect(cfg.clientId.c_str(), cfg.user.c_str(), cfg.pass.c_str());
//   } else {
//     ok = mqtt.connect(cfg.clientId.c_str());
//   }

//   if (ok) {
//     Serial.println("MQTT connected!");
//     if (cfg.topicSub.length() > 0) {
//       mqtt.subscribe(cfg.topicSub.c_str());
//       Serial.printf("Subscribed to %s\n", cfg.topicSub.c_str());
//     }
//   } else {
//     Serial.printf("MQTT connect failed, rc=%d\n", mqtt.state());
//   }
//   return ok;
// }

// void mqttPublish(const char* topic, const String& payload) {
//   if (!mqtt.connected()) return;
//   mqtt.publish(topic, payload.c_str());
//   Serial.printf("[MQTT OUT] %s => %s\n", topic, payload.c_str());
// }

// void mqttSubscribe(const char* topic) {
//   if (!mqtt.connected()) return;
//   mqtt.subscribe(topic);
//   Serial.printf("[MQTT SUB] %s\n", topic);
// }