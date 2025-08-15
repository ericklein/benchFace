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

// Status variables shared across various functions

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

String generateTopic(char *key)
// Utility function to streamline dynamically generating MQTT topics using site and device 
// parameters defined in config.h and our standard naming scheme using values set in secrets.h
{
  String topic;
  topic = String(DEVICE_SITE) + "/" + String(DEVICE_LOCATION) + "/" + String(DEVICE_ROOM) +
          "/" + String(DEVICE) + "/" + String(DEVICE_ID) + "/" + String(key);
  debugMessage(String("Generated MQTT topic: ") + topic,2);
  return(topic);
}

bool mqttDeviceWiFiUpdate(uint8_t rssi)
{
  bool published = false;
  String topic;
  topic = generateTopic(VALUE_KEY_RSSI);  // Generate topic using config.h and secrets.h parameters
  // add ,MQTT_QOS_1); if problematic, remove QOS parameter
  Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&bl_mqtt, topic.c_str());
  
  mqttConnect();

  if (rssiLevelPub.publish((uint32_t)rssi)) // explicitly cast due to compiler ambiguity
  {
    debugMessage("MQTT publish: WiFi RSSI succeeded",2);
    published = true;
  }
  else
    debugMessage("MQTT publish: WiFi RSSI failed",2);
  return(published);
}

bool mqttDeviceLightUpdate(bool status)
{
  bool published = false;
  String topic;
  topic = generateTopic(VALUE_KEY_LIGHT);  // Generate topic using config.h and secrets.h parameters
  // add ,MQTT_QOS_1); if problematic, remove QOS parameter
  Adafruit_MQTT_Publish lightPub = Adafruit_MQTT_Publish(&bl_mqtt, topic.c_str());
  
  mqttConnect();

  if (lightPub.publish((uint32_t)status))
  {
    debugMessage("MQTT publish: light status succeeded",1);
    published = true;
  }
  else
    debugMessage("MQTT publish: light status failed",1);
  return(published);
}

uint8_t mqttBenchLightMessage()
{
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = bl_mqtt.readSubscription(5000))) 
  {
    if (subscription == &benchLightSub)
    {
      debugMessage(String("Received '") + ((char *)benchLightSub.lastread) + "' from " + MQTT_SUB_TOPIC,1); 
      return (atol((char *)benchLightSub.lastread) == 1) ? 1 : 0;
    }
  }
  return 2;  // 2 indicates no message received
}