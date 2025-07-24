/*
  Project Name:   benchFace
  Description:    trigger (bench) light via CV or MQTT
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// library for Useful Objects People Sensor
#include <Wire.h>
#include "person_sensor.h"

#if defined (ESP8266)
  #include <ESP8266WiFi.h>
#elif defined (ESP32)
  #include <WiFi.h>
#endif

// MQTT setup
// MQTT uses WiFiClient class to create TCP connections
WiFiClient client;

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
Adafruit_MQTT_Client bl_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);
extern Adafruit_MQTT_Subscribe benchLightSub;
extern bool mqttConnect();
extern bool mqttDeviceWiFiUpdate(uint8_t rssi);
extern bool mqttDeviceLightUpdate(bool status);
extern uint8_t mqttBenchLightMessage();

// Global variables
// timers
uint32_t timeLastMQTTPingMS = 0; 
uint32_t timeLastSensorSampleMS = 0;
uint32_t timeLastFaceSeenMS = 0;
uint32_t timeLastWiFiConnectMS = 0;

uint8_t rssi = 0; // 0 value used to indicate no WiFi connection
bool faceSeen = false;

void setup() {
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) ;
    debugMessage("benchLight started",1);
    debugMessage(String("Client ID: ") + DEVICE_ID,1);
  #endif

  Wire.begin();

  pinMode(hardwareRelayPin, OUTPUT);

  // Setup network connection specified in secrets.h
  if (!networkConnect())
  {
    // alert user to the WiFi connectivity problem
    debugMessage(String("Connection to ") + WIFI_SSID + " failed", 1);
    // IMPROVEMENT: How do we want to handle this? the rest of the code will barf...
    // ESP.restart();
  }

  mqttDeviceWiFiUpdate(rssi);
  bl_mqtt.subscribe(&benchLightSub);
}

void loop() {

  // re-establish WiFi connection if needed
  if ((WiFi.status() != WL_CONNECTED) && (millis() - timeLastWiFiConnectMS > timeWiFiKeepAliveIntervalMS)) {
    timeLastWiFiConnectMS = millis();
    if (!networkConnect()) {
      // alert user to the WiFi connectivity problem
      rssi = 0;
      debugMessage(String("Connection to ") + WIFI_SSID + " failed", 1);
      // IMPROVEMENT: How do we want to handle this? the rest of the code will barf...
      // ESP.restart();
    }
    mqttDeviceWiFiUpdate(rssi);
  }

  // is there a MQTT message to process?
  // ensure we have a connection to MQTT
  if (!mqttConnect())
  {
    // error; not sure how to handle
  }
  else
  {
    // check to see if there is a status change for the light
    uint8_t status = mqttBenchLightMessage();

    if (status != 2) { // 2 is no message received 
      debugMessage(String("MQTT; change light to ") + (status == 1 ? "On" : "Off"),1);
      digitalWrite(hardwareRelayPin, status);
      if (!status)
        faceSeen = false; // allows for a person to be detected after a remote override of light state
    }
    // Question : Do we need to ping the server if we are already querying it regularly?
    // if(! bl_mqtt.ping()) {
    //   mqtt.disconnect();
  }

  // is it time to look for a face?
  if((millis() - timeLastSensorSampleMS) >= sensorSampleIntervalMS)
  {
    person_sensor_results_t results = {};
    if (!person_sensor_read(&results))
      // error, not sure how to handle this yet...
      debugMessage("No person sensor results found on the i2c bus",1);
    else {
      if (results.num_faces>0) {
        debugMessage(String(results.num_faces) + " face(s) detected",1);
        for (uint8_t loop = 0; loop < results.num_faces; ++loop) {
          const person_sensor_face_t* face = &results.faces[loop];
          debugMessage(String("confidence in face ") + loop + " is " + face->box_confidence + "%",2);
          debugMessage(String("face ") + loop + " bounding box is (" + face->box_left + ", " + face->box_top + "), (" + face->box_right + ", " + face->box_bottom +
            + ")",2);
          if (face->is_facing)
            debugMessage(String("face ") + loop + " is facing camera",2);
          else
            debugMessage(String("face ") + loop + " is not facing camera",2);
        }
        if (!faceSeen) {
          digitalWrite(hardwareRelayPin, HIGH);
          mqttDeviceLightUpdate(true);
        }
        faceSeen = true;
        timeLastFaceSeenMS = millis();
      }
    }
    timeLastSensorSampleMS = millis();
  }

  // have we seen a face inside the timeout window?
  if (((millis() - timeLastFaceSeenMS) > faceDetectTimeoutWindowMS) && (faceSeen)) {
    debugMessage(String("No face seen in ") + (faceDetectTimeoutWindowMS/1000) + " seconds, turning off light",1);
    digitalWrite(hardwareRelayPin, LOW);
    mqttDeviceLightUpdate(false);
    faceSeen = false;
  }
}

bool networkConnect() 
// Connect to WiFi network specified in secrets.h
{
  // reconnect to WiFi only if needed
  if (WiFi.status() == WL_CONNECTED) 
  {
    debugMessage("Already connected to WiFi",2);
    return true;
  }

  WiFi.mode(WIFI_STA); // IMPROVEMENT: test to see if this improves connection time
  // set hostname has to come before WiFi.begin
  WiFi.hostname(DEVICE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t timeWiFiConnectStart = millis();
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - timeWiFiConnectStart) < timeNetworkConnectTimeoutMS)) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) 
    {
      rssi = abs(WiFi.RSSI());
      debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString(), 1);
      debugMessage(String("WiFi RSSI is: ") + rssi + " dBm", 2);
      return true;
    }
  else {
    return false;
  }
}

void debugMessage(String messageText, uint8_t messageLevel)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  if (messageLevel <= DEBUG) {
    Serial.println(messageText);
    Serial.flush();  // Make sure the message gets output before other functions
  }
#endif
}