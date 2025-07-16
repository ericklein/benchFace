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
uint32_t timeLastMQTTPingTime = 0;
uint32_t timeLastSensorSample = 0;
uint32_t timeLastFaceSeen = 0;

uint8_t rssi = 0; // 0 value used to indicate no WiFi connection

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

  // Setup network connection specified in config.h
  if (!networkConnect())
  {
    // alert user to the WiFi connectivity problem
    debugMessage("unable to connect to WiFi",1);
    //ESP.restart();
  }
  
  bl_mqtt.subscribe(&benchLightSub);
}

void loop() {

/* is it time to look for a face?
 yes - did I find one?
  yes - turn on the lamp, update MQTT to on, update last_face_seen
  no - move along in current state
 no - move along in current state
no
 check to see if there is a MQTT message for the light
   yes  - is the message for light on?
     yes - turn it on if it wasn't already, update last_face_seen
     no - turn the light off # if someone is there, it will immediately turn back on
   no - move along in current state
have we seen a face in the specified max window?
  yes - move along in curren state
  no - turn off the lamp, update MQTT to off
Q - could we ESP light sleep here for a second or two?
*/

  // is it time to look for a face?
  if((millis() - timeLastSensorSample) >= (sensorSampleInterval * 1000)) // converting sensorSampleInterval into milliseconds
  {
    person_sensor_results_t results = {};
    if (!person_sensor_read(&results))
      // error, not sure how to handle this yet...
      debugMessage("No person sensor results found on the i2c bus",1);
    else {
      debugMessage(String(results.num_faces) + " faces detected",1);
      for (uint8_t loop = 0; loop < results.num_faces; ++loop) {
        const person_sensor_face_t* face = &results.faces[loop];
        debugMessage(String("face #") + loop + ": " + face->box_confidence + " confidence (" +
          face->box_left + ", " + face->box_top + "), (" + face->box_right + ", " + face->box_bottom +
          + ")",2);
        if (face->is_facing)
          debugMessage("facing",2);
        else
          debugMessage("not facing",2);
      digitalWrite(hardwareRelayPin, HIGH);
      mqttDeviceWiFiUpdate(rssi);
      mqttDeviceLightUpdate(true);
      timeLastFaceSeen = millis();
      }
    }
    timeLastSensorSample = millis();
  }
  // is there a MQTT message to process?
  // ensure we have a connection to MQTT
  if (!mqttConnect())
  {
    // error; not sure how to handle
  }
  else
  {
    // keep the MQTT broker connection active for subscription via ping
    if((millis() - timeLastMQTTPingTime) > (networkMQTTKeepAliveInterval * 1000))
    {
      timeLastMQTTPingTime = millis();   
      if(bl_mqtt.ping())
        debugMessage("MQTT broker pinged to maintain connection",1);
      else
      {
        debugMessage("unable to ping MQTT broker; disconnected",1);
        bl_mqtt.disconnect();
      }
    }
    // check to see if there is a status change for the light
    uint8_t status = mqttBenchLightMessage();

    if (status != 2) { // 2 is no message received 
      debugMessage(String("MQTT message; change light to ") + (status == 1 ? "On" : "Off"),1);
      digitalWrite(hardwareRelayPin, status);
      timeLastFaceSeen = millis(); // not really true but resets timeout window
    }
  }
  // have we seen a face before the timeout window?
  if((millis() - timeLastFaceSeen) > (faceDetectTimeoutWindow * 1000)) {
    debugMessage(String("No face seen in ") + faceDetectTimeoutWindow + " seconds, turning off light",1);
    digitalWrite(hardwareRelayPin, LOW);
    mqttDeviceWiFiUpdate(rssi);
    mqttDeviceLightUpdate(false);
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
  // set hostname has to come before WiFi.begin
  WiFi.hostname(DEVICE_ID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (uint8_t loop = 1; loop <= networkConnectAttemptLimit; loop++)
  // Attempts WiFi connection, and if unsuccessful, re-attempts after networkConnectAttemptInterval second delay for networkConnectAttemptLimit times
  {
    if (WiFi.status() == WL_CONNECTED) 
    {
      rssi = abs(WiFi.RSSI());
      debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString(), 1);
      debugMessage(String("WiFi RSSI is: ") + rssi + " dBm", 1);
      return true;
    }
    debugMessage(String("Connection attempt ") + loop + " of " + networkConnectAttemptLimit + " to " + WIFI_SSID + " failed", 1);
    // use of delay() OK as this is initialization code
    delay(networkConnectAttemptInterval * 1000);  // converted into milliseconds
  }
  return false;
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