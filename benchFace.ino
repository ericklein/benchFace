/*
  Project Name:   benchFace
  Description:    toggle (bench) light via CV or MQTT message
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// library for Useful Objects People Sensor
#include <Wire.h>
#include "person_sensor.h"

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
WiFiManager wfm;

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvConfigStorage;

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

// flag to let us know we need to save config data from WiFi Manager AP mode
bool saveWFMConfig = false;

String hardwareDeviceSite;
String hardwareDeviceLocation;
String hardwareDeviceRoom;
String hardwareDeviceID;

// Question: does saveConfigCallback() need to be in front of setup()?

void setup() {
  // config Serial first for debugMessage()
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
    debugMessage("benchLight started", 1);
  #endif

  // configure relay pin
  Wire.begin();
  pinMode(hardwareRelayPin, OUTPUT);

  WiFi.hostname(DEVICE_ID);

  // configure WiFiManager
  #ifndef DEBUG
    wfm.SetDebugOutput(false);
  #endif
  wfm.setConnectTimeout(180);
  wfm.setConnectRetries(100);
  wfm.resetSettings(); // wipe stored credentials

   // hint text (optional)
  //WiFiManagerParameter hint_text("<small>*If you want to connect to already connected AP, leave SSID and password fields empty</small>");
  
  // collect MQTT and device parameters while in AP mode
  // WiFiManagerParameter mqttBroker("mqttBroker","MQTT broker address","192.168.1.27",30);;
  // WiFiManagerParameter mqttPort("mqttPort", "MQTT broker port", "1883", 5);
  // WiFiManagerParameter mqttUser("mqttUser", "MQTT username", "eric", 20);
  // WiFiManagerParameter mqttPassword("mqttPassword", "MQTT user password", "default", 20);

  // collect parameters used to build network endpoint paths
  WiFiManagerParameter deviceSite("deviceSite", "device site", "7828", 20);
  WiFiManagerParameter deviceLocation("deviceLocation", "indoor or outdoor", "indoor", 20);
  WiFiManagerParameter deviceRoom("deviceRoom", "what room is the device in", "bedroom", 20);
  WiFiManagerParameter deviceID("deviceID", "unique name for device", "benchLight 001" , 30);
 
  // order determines on-screen order
  //wfm.addParameter(&hint_text);
  // wfm.addParameter(&mqttBroker);
  // wfm.addParameter(&mqttPort);
  // wfm.addParameter(&mqttUser);
  // wfm.addParameter(&mqttPassword);
  wfm.addParameter(&deviceSite);
  wfm.addParameter(&deviceLocation);
  wfm.addParameter(&deviceRoom);
  wfm.addParameter(&deviceID);

  //set config save notify callback
  wfm.setSaveConfigCallback(saveConfigCallback);

  if(networkConnect()) {
      // open config prefs in read-write mode
      nvConfigStorage.begin("benchLight", false);
      if (saveWFMConfig) {
      // copy new config data to non-volatile storage
      hardwareDeviceSite = deviceSite.getValue();
      nvConfigStorage.putString("deviceSite", hardwareDeviceSite);

      hardwareDeviceLocation = deviceLocation.getValue();
      nvConfigStorage.putString("deviceLocation", hardwareDeviceLocation);

      hardwareDeviceRoom = deviceRoom.getValue();
      nvConfigStorage.putString("deviceRoom", hardwareDeviceRoom);

      hardwareDeviceID = deviceID.getValue();
      nvConfigStorage.putString("deviceID", hardwareDeviceID);
    }
    // read config data from non-volatile storage
    if (!hardwareDeviceSite){
      hardwareDeviceSite = nvConfigStorage.getString("deviceSite");
      debugMessage(String("Device site is ") + hardwareDeviceSite,2);
      hardwareDeviceLocation = nvConfigStorage.getString("deviceLocation");
      debugMessage(String("Device location is ") + hardwareDeviceLocation,2);
      hardwareDeviceRoom = nvConfigStorage.getString("deviceRoom");
      debugMessage(String("Device room is ") + hardwareDeviceRoom,2);
      hardwareDeviceID = nvConfigStorage.getString("deviceID");
      debugMessage(String("Device ID is ") + hardwareDeviceID,1);
    }
    // now that network endpoint path parameters loaded, write away :)
    mqttDeviceWiFiUpdate(abs(WiFi.RSSI()));
    bl_mqtt.subscribe(&benchLightSub); // IMPROVEMENT: Should this be also implemented in loop() in case WiFi connection is established later?
  }
}

void loop() {
  // re-establish WiFi connection if needed
  if ((WiFi.status() != WL_CONNECTED) && (millis() - timeLastWiFiConnectMS > timeWiFiKeepAliveIntervalMS)) {
    bool connected;

    timeLastWiFiConnectMS = millis();
    if (wfm.getWiFiIsSaved()) 
      wfm.setEnableConfigPortal(false);
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

  // connected = wfm.autoConnect(); // auto generated AP name from chipid
  connected = wfm.autoConnect("benchLight AP"); // anonymous ap
  // connected = wfm.autoConnect("AutoConnectAP","password"); // password protected ap

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

void saveConfigCallback() 
//callback notifying us of the need to save config from WiFi Manager AP mode
{
  debugMessage("Need to save config info from WiFi Manager AP mode",1);
  saveWFMConfig = true;
}