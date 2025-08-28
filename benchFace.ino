/*
  Project Name:   benchFace
  Description:    toggle (bench) light via CV or MQTT message
*/

#include "benchFace.h"        // core data definitions across files
#include "config.h"           // hardware and internet configuration parameters
#include "secrets.h"          // private credentials for network, MQTT
#include <Wire.h>             
#include <Preferences.h>      // read-write to ESP32 persistent storage
#include "person_sensor.h"    // https://github.com/moonshine-ai/person_sensor_arduino
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>     // https://github.com/knolleary/pubsubclient

WiFiClient client;  // WiFiManager loads WiFi.h
Preferences nvConfig;
PubSubClient mqtt(client);

extern bool mqttConnect();
extern void mqttPublish(const char* topic, const String& payload);
extern void mqttSubscribe(const char* topic);
extern void mqttMessageCallback(char* topic, byte* payload, unsigned int length);
extern const char* generateMQTTTopic(String key);

// Global variables
// timers
uint32_t timeLastMQTTPingMS = 0; 
uint32_t timeLastSensorSampleMS = 0;
uint32_t timeLastFaceSeenMS = 0;
uint32_t timeResetPressStartMS = 0;

MqttConfig mqttBrokerConfig;
networkEndpointConfig endpointPath;

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
  
  loadNVConfig();

  if (openWiFiManager()) {
    // setup MQTT
    mqtt.setCallback(mqttMessageCallback);
    if (mqttConnect()) {
      // publish new WiFi RSSI to MQTT
      const char* topic = generateMQTTTopic(VALUE_KEY_RSSI);
      mqttPublish(topic, String(abs(WiFi.RSSI())));

      // subscribe to MQTT broker light topic
      const char* lightTopic = generateMQTTTopic(VALUE_KEY_LIGHT);
      mqttSubscribe(lightTopic);
    }
  }
}

void loop() {
  checkResetLongPress();  // Always watching for long-press to wipe

  // if (wifiManager.getWiFiIsSaved()) 
  //   wifiManager.setEnableConfigPortal(false); 
  // wifiManager.autoConnect("benchLight AP");

  // Maintain MQTT connection
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - timeLastMQTTPingMS > timeMQTTKeepAliveIntervalMS) {
      timeLastMQTTPingMS = now;
      mqttConnect();
    }
  } else {
    mqtt.loop();
  }

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
          const char* lightTopic = generateMQTTTopic(VALUE_KEY_LIGHT);
          mqttPublish(lightTopic, String(true));
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
    const char* lightTopic = generateMQTTTopic(VALUE_KEY_LIGHT);
    mqttPublish(lightTopic, String(false));
    faceSeen = false;
  }
}

// WiFiManager portal functions
void saveConfigCallback() 
//callback notifying us of the need to save config from WiFi Manager AP mode
{
  saveWFMConfig = true;
}

bool openWiFiManager()
// Connect to WiFi network using WiFiManager
{
  bool connected;

  debugMessage("openWiFiManager begin",2);

  WiFiManager wfm;

  wfm.setSaveConfigCallback(saveConfigCallback);
  wfm.setHostname(endpointPath.deviceID.c_str());
  #ifndef DEBUG
    wfm.SetDebugOutput(false);
  #endif
  wfm.setConnectTimeout(180);

  String titleText = hardwareDeviceType + " setup";
  wfm.setTitle(titleText);
  // hint text (optional)
  //WiFiManagerParameter hint_text("<small>*If you want to connect to already connected AP, leave SSID and password fields empty</small>");
  
  // collect MQTT and device parameters while in AP mode
  WiFiManagerParameter mqttBroker("mqttBroker","MQTT broker address",defaultMQTTBroker.c_str(),30);;
  WiFiManagerParameter mqttPort("mqttPort", "MQTT broker port", defaultMQTTPort.c_str(), 5);
  WiFiManagerParameter mqttUser("mqttUser", "MQTT username", defaultMQTTUser.c_str(), 20);
  WiFiManagerParameter mqttPassword("mqttPassword", "MQTT user password", defaultMQTTPassword.c_str(), 20);

  // collect parameters used to build network endpoint paths
  WiFiManagerParameter deviceSite("deviceSite", "device site", defaultSite.c_str(), 20);
  WiFiManagerParameter deviceLocation("deviceLocation", "indoor or outdoor", defaultLocation.c_str(), 20);
  WiFiManagerParameter deviceRoom("deviceRoom", "what room is the device in", defaultRoom.c_str(), 20);
  WiFiManagerParameter deviceID("deviceID", "unique name for device", defaultDeviceID.c_str(), 30);
 
  //order determines on-screen order
  // wfm.addParameter(&hint_text);
  wfm.addParameter(&mqttBroker);
  wfm.addParameter(&mqttPort);
  wfm.addParameter(&mqttUser);
  wfm.addParameter(&mqttPassword);

  wfm.addParameter(&deviceSite);
  wfm.addParameter(&deviceLocation);
  wfm.addParameter(&deviceRoom);
  wfm.addParameter(&deviceID);

  connected = wfm.autoConnect("benchLight AP"); // anonymous ap
  // connected = wfm.autoConnect("benchLight AP","password"); // password protected AP

  if(!connected) {
    debugMessage("WiFi connection failure; local light control ONLY", 1);
    // ESP.restart(); // if MQTT support is critical, make failure a stop gate
  } 
  else {
    if (saveWFMConfig) {
      debugMessage("retreiving new parameters from AP portal",2);
      endpointPath.site = deviceSite.getValue();
      endpointPath.location = deviceLocation.getValue();
      endpointPath.room = deviceRoom.getValue();
      endpointPath.deviceID = deviceID.getValue();

      mqttBrokerConfig.host     = mqttBroker.getValue();
      mqttBrokerConfig.port     = (uint16_t)strtoul(mqttPort.getValue(), nullptr, 10);
      mqttBrokerConfig.user     = mqttUser.getValue();
      mqttBrokerConfig.password = mqttPassword.getValue();

      saveNVConfig();
      saveWFMConfig = false;
    }
    debugMessage(String("OpenWiFiManager end; connected to " + WiFi.SSID() + ", ") + abs(WiFi.RSSI()) + " dBm RSSI", 1);
  }
  return (connected);
}

// Preferences helper routines
void loadNVConfig() {
  debugMessage("loadNVConfig begin",2);
  nvConfig.begin("config", true); // read-only

  endpointPath.site = nvConfig.getString("site", defaultSite);
  debugMessage(String("Device site is ") + endpointPath.site,2);
  endpointPath.location = nvConfig.getString("location", defaultLocation);
  debugMessage(String("Device location is ") + endpointPath.location,2);
  endpointPath.room = nvConfig.getString("room", defaultRoom);
  debugMessage(String("Device room is ") + endpointPath.room,2);
  endpointPath.deviceID = nvConfig.getString("deviceID", defaultDeviceID);
  debugMessage(String("Device ID is ") + endpointPath.deviceID,1);

  mqttBrokerConfig.host     = nvConfig.getString("host", defaultMQTTBroker);
  debugMessage(String("MQTT broker is ") + mqttBrokerConfig.host,2);
  // FIX THIS: Just putting a bandaid on this so I can work on the bigger shit
  // mqttBrokerConfig.port     = nvConfig.getUShort("port", ((uint16_t)strtoul(defaultMQTTPort, nullptr, 10)));
  mqttBrokerConfig.port     = nvConfig.getUShort("port", 1883);
  debugMessage(String("MQTT broker port is ") + mqttBrokerConfig.port,2);
  mqttBrokerConfig.user     = nvConfig.getString("user", defaultMQTTUser);
  debugMessage(String("MQTT username is ") + mqttBrokerConfig.user,2);
  mqttBrokerConfig.password = nvConfig.getString("password", defaultMQTTPassword);
  debugMessage(String("MQTT user password is ") + mqttBrokerConfig.password,2);

  nvConfig.end();
  debugMessage("loadNVConfig end",2);
}

void saveNVConfig()
// copy new config data to non-volatile storage
{
  debugMessage("saveNVConfig begin",2);
  nvConfig.begin("config", false); // read-write

  nvConfig.putString("site", endpointPath.site);
  nvConfig.putString("location", endpointPath.location);
  nvConfig.putString("room", endpointPath.room);
  nvConfig.putString("deviceID", endpointPath.deviceID);

  nvConfig.putString("host",  mqttBrokerConfig.host);
  nvConfig.putUShort("port",  mqttBrokerConfig.port);
  nvConfig.putString("user",  mqttBrokerConfig.user);
  nvConfig.putString("password",  mqttBrokerConfig.password);

  nvConfig.end();
  debugMessage("saveNVConfig end",2);
}

void wipePrefsAndReboot() 
// Wipes all ESP, WiFiManager preferences and reboots device
{
  debugMessage("wipePrefsAndReboot begin",2);

  // Clear nv storage
  nvConfig.begin("config", false);
  nvConfig.clear();
  nvConfig.end();

  // disconnect and clear (via true) stored Wi-Fi credentials
  WiFi.disconnect(true);

  // Clear WiFiManager settings (AP config)
  WiFiManager wm;
  wm.resetSettings();

  debugMessage("wipePrefsAndReboot end, rebooting...",2);
  ESP.restart();
}

void checkResetLongPress() {
  uint8_t buttonState = digitalRead(hardwareWipeButton);

  if (buttonState == LOW) {
    debugMessage(String("button pressed for ") + ((millis() - timeResetPressStartMS)/1000) + " seconds",2);
    if (timeResetPressStartMS == 0)
      timeResetPressStartMS = millis();
    if (millis() - timeResetPressStartMS >= timeResetButtonHoldMS)
      wipePrefsAndReboot();
  }
  else {
    if (timeResetPressStartMS != 0) {
      debugMessage("button released",2);
      timeResetPressStartMS = 0; // reset button press timer
    }
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