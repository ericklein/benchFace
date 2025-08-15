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

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
WiFiManager wm;

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

bool faceSeen = false;

void setup() {
  // config Serial first for debugMessage()
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
    debugMessage("benchLight started", 1);
    debugMessage(String("Client ID: ") + DEVICE_ID, 1);
  #endif

  // configure relay pin
  Wire.begin();
  pinMode(hardwareRelayPin, OUTPUT);

  WiFi.hostname(DEVICE_ID);

  // configure WiFiManager
  #ifndef DEBUG
    wm.SetDebugOutput(false);
  #endif
  wm.setConnectTimeout(180);
  wm.setConnectRetries(100);
  // wm.resetSettings(); // wipe stored credentials

  if(networkConnect()) {
    mqttDeviceWiFiUpdate(abs(WiFi.RSSI()));
    bl_mqtt.subscribe(&benchLightSub); // IMPROVEMENT: Should this be also implemented in loop() in case WiFi connection is established later?
  }
}

void loop() {

  // re-establish WiFi connection if needed
  if ((WiFi.status() != WL_CONNECTED) && (millis() - timeLastWiFiConnectMS > timeWiFiKeepAliveIntervalMS)) {
    bool connected;

    timeLastWiFiConnectMS = millis();
    if (wm.getWiFiIsSaved()) 
      wm.setEnableConfigPortal(false);
    if(networkConnect()) {
      mqttDeviceWiFiUpdate(abs(WiFi.RSSI()));
      bl_mqtt.subscribe(&benchLightSub); // IMPROVEMENT: Should this be also implemented in loop() in case WiFi connection is established later?
    }
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
// Connect to WiFi network using WiFi Manager library
{
  bool connected;

  // connected = wm.autoConnect(); // auto generated AP name from chipid
  connected = wm.autoConnect("benchLight AP"); // anonymous ap
  // connected = wm.autoConnect("AutoConnectAP","password"); // password protected ap

  if(!connected) {
    debugMessage("Failed to connect to WiFi, local control of light ONLY", 1);
    // ESP.restart(); // if MQTT support is critical, make failure a stop gate
  } 
  else
    debugMessage(String("WiFi RSSI is: ") + abs(WiFi.RSSI()) + " dBm", 1);
  return (connected);
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