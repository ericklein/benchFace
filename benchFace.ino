/*
  Project Name:   benchFace
  Description:    toggle (bench) light via CV or MQTT message
*/

#include "config.h"           // hardware and internet configuration parameters
#include "secrets.h"          // private credentials for network, MQTT
#include <Wire.h>             
#include <Preferences.h>      // read-write to ESP32 persistent storage
#include "person_sensor.h"    // https://github.com/moonshine-ai/person_sensor_arduino
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
// new MQTT
// #include <PubSubClient.h>  // https://github.com/knolleary/pubsubclient

WiFiClient client;  // WiFiManager loads WiFi.h
Preferences nvConfig;
// PubSubClient mqtt(espClient);

// MQTT setup
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
Adafruit_MQTT_Client bl_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);
extern Adafruit_MQTT_Subscribe benchLightSub;
extern bool mqttConnect();
extern bool mqttDeviceWiFiUpdate(uint32_t rssi);
extern bool mqttDeviceLightUpdate(bool status);
extern uint8_t mqttBenchLightMessage();

// Global variables
// timers
uint32_t timeLastMQTTPingMS = 0; 
uint32_t timeLastSensorSampleMS = 0;
uint32_t timeLastFaceSeenMS = 0;
uint32_t timeResetPressStartMS = 0;

// in config.h
struct MqttConfig mqttConfig;
struct networkEndpointConfig endpointPath; 

bool faceSeen = false;
bool saveWFMConfig = false;

void setup() {
  // config Serial first for debugMessage()
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
    debugMessage("benchLight started", 1);
  #endif

  // configure hardware pins
  Wire.begin();
  pinMode(hardwareRelayPin, OUTPUT);
  pinMode(hardwareWipeButton, INPUT_PULLUP);

  // initiate device wipe at boot if button is held
  for (uint8_t i = 0; i < 50; i++) { // ~500ms grace window
    checkResetLongPress();
    delay(10);
  }
  
  loadNVConfig();

  if (needPortal()) {
    openWiFiManager();     // Handles Wi-Fi + hostname + MQTT param entry
  } else {
    WiFi.begin();          // Use stored Wi-Fi creds
  }

  // new MQTT 
  // mqtt.setCallback(mqttMessageCallback);
  // mqtt.setServer(cfg.host.c_str(), cfg.port);

  mqttDeviceWiFiUpdate(abs(WiFi.RSSI()));
  bl_mqtt.subscribe(&benchLightSub); // IMPROVEMENT: Should this be also implemented in loop() in case WiFi connection is established later?
}

void loop() {
  checkResetLongPress();  // Always watching for long-press to wipe

  // Keep Wi-Fi alive; if not connected, wait and retry passively
  if (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    return;
  }

  // new MQTT 
  //   // Maintain MQTT connection
  // if (!mqtt.connected()) {
  //   unsigned long now = millis();
  //   if (now - lastMqttAttempt > MQTT_RECONNECT_MS) {
  //     lastMqttAttempt = now;
  //     mqttConnect();
  //   }
  // } else {
  //   mqtt.loop();
  // }

  // // Example: publish every 5 seconds when connected
  // static unsigned long lastPub = 0;
  // if (mqtt.connected() && millis() - lastPub > 5000) {
  //   lastPub = millis();
  //   mqttPublish(cfg.topicPub.c_str(), "hello from ESP32");
  // }

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

// WiFiManager portal functions
void saveConfigCallback() 
//callback notifying us of the need to save config from WiFi Manager AP mode
{
  debugMessage("Need to save config info from WiFi Manager AP mode",1);
  saveWFMConfig = true;
}

bool needPortal() {
  if (digitalRead(hardwareWipeButton) == LOW) return true; // force portal at boot

  loadNVConfig();
  return mqttConfig.host.length() == 0; // no MQTT host saved → need portal
}

bool openWiFiManager()
// Connect to WiFi network using WiFiManager
{
  bool connected;

  WiFiManager wfm;

  wfm.setSaveConfigCallback(saveConfigCallback);
  wfm.setHostname(endpointPath.deviceID.c_str());
  #ifndef DEBUG
    wfm.SetDebugOutput(false);
  #endif
  wfm.setConnectTimeout(180);
  wfm.setConnectRetries(100);

  wfm.setTitle("setup benchLight device");
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

  connected = wfm.autoConnect("benchLight AP"); // anonymous ap
  // connected = wfm.autoConnect("benchLight AP","password"); // password protected AP

  if(!connected) {
    debugMessage("Failed to connect to WiFi, local control of light ONLY", 1);
    // ESP.restart(); // if MQTT support is critical, make failure a stop gate
  } 
  else {
    if (saveWFMConfig) {
      endpointPath.site = deviceSite.getValue();
      endpointPath.location = deviceLocation.getValue();
      endpointPath.room = deviceRoom.getValue();
      endpointPath.deviceID = deviceID.getValue();

      // new MQTT
      // mqttBrokerConfig.host     = mqttBroker.getValue();
      // mqttBrokerConfig.port     = (uint16_t)strtoul(mqttPort.getValue(), nullptr, 10);
      // mqttBrokerConfig.user     = mqttUser.getValue();
      // mqttBrokerConfig.pass     = mqttPassword.getValue();

      saveNVConfig();
      saveWFMConfig = false;
      debugMessage("saved portal parameters to nv storage",1);
      debugMessage(String("Connected to WiFI AP " + WiFi.SSID() + " with ") + abs(WiFi.RSSI()) + " dBm RSSI", 1);
    }
  }
  return (connected);
}

// Preferences helper routines
void loadNVConfig() {
  nvConfig.begin("config", true); // read-only

  endpointPath.site = nvConfig.getString("site", defaultSite);
  debugMessage(String("Device site is ") + endpointPath.site,2);
  endpointPath.location = nvConfig.getString("location", defaultLocation);
  debugMessage(String("Device location is ") + endpointPath.location,2);
  endpointPath.room = nvConfig.getString("room", defaultRoom);
  debugMessage(String("Device room is ") + endpointPath.room,2);
  endpointPath.deviceID = nvConfig.getString("deviceID", defaultDeviceID);
  debugMessage(String("Device ID is ") + endpointPath.deviceID,1);

  // new MQTT
  // mqttBrokerConfig.host     = nvConfig.getString("host", defaultMQTTBroker);
  // mqttBrokerConfig.port     = nvConfig.getUShort("port", defaultMQTTPort);
  // mqttBrokerConfig.user     = nvConfig.getString("user", defaultMQTTUser);
  // mqttBrokerConfig.password = nvConfig.getString("password", defaultMQTTPassword);
  nvConfig.end();
}

void saveNVConfig()
//void saveNVConfig(const MqttConfig& config)
// copy new config data to non-volatile storage
{
  nvConfig.begin("config", false); // read-write

  nvConfig.putString("site", endpointPath.site);
  nvConfig.putString("location", endpointPath.location);
  nvConfig.putString("room", endpointPath.room);
  nvConfig.putString("deviceID", endpointPath.deviceID);

  // new MQTT
  // nvConfig.putString("host",  mqttBrokerConfig.host);
  // nvConfig.putUShort("port",  mqttBrokerConfig.port);
  // nvConfig.putString("user",  mqttBrokerConfig.user);
  // nvConfig.putString("password",  mqttBrokerConfig.password);

  nvConfig.end();
  debugMessage("New config information saved to nv storage",1);
}

void wipePrefsAndReboot() 
// Wipes all ESP, WiFiManager preferences and reboots device
{
  debugMessage("Wiping all device preferences and rebooting",1);

  // Clear nv storage
  nvConfig.begin("config", false);
  nvConfig.clear();
  nvConfig.end();

  // disconnect and clear (via true) stored Wi-Fi credentials
  WiFi.disconnect(true);

  // Clear WiFiManager settings (AP config)
  WiFiManager wm;
  wm.resetSettings();

  delay(200);
  ESP.restart();
}

void checkResetLongPress() {
  uint8_t level = digitalRead(hardwareWipeButton);

  if (level == LOW) {
    if (timeResetPressStartMS == 0) timeResetPressStartMS = millis();
    if (millis() - timeResetPressStartMS >= timeResetButtonHoldMS)
      wipePrefsAndReboot();
    else
      timeResetPressStartMS = 0; // released
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