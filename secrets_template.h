/*
  Project Name:   benchFace
  Description:    private configuration data template that needs to be saved as secrets.h after github cloning the project
*/

// Configuration Step 1: Set default network endpoint and mqtt broker parameters. These will only
// be used if the user doesn't enter then in the configuration AP portal.

const String defaultSite = "value";            // physical address of the device, e.g. "1234 Main"
const String defaultLocation = "value";        // general location of device at physical address, e.g. "indoor"
const String defaultRoom = "value";            // specific location of device within location, e.g. "kitchen"
const String defaultDeviceType = "benchLight"; // name of all device like this, don't need to change per device
const String defaultDeviceID = defaultDeviceType + "-" + String((uint32_t)ESP.getEfuseMac(), HEX); 

const String defaultMQTTBroker = "192.168.1.1"; // mqtt.hostname.local or IP address
const String defaultMQTTPort = "1883";          // use 8883 for SSL (codepath not tested!)
const String defaultMQTTUser = "username";      // if needed by MQTT broker
const String defaultMQTTPassword = "password";  // if needed by MQTT broker

// descriptors for the actual data key pair
#define VALUE_KEY_LIGHT   "key_value" // "light"
#define VALUE_KEY_RSSI    "key_value" // "rssi"