/*
  Project Name:   benchFace
  Description:    private configuration data template that needs to be saved as secrets.h after github cloning the project
*/

// Configuration Step 1: Set WiFi credentials
// set the WiFi SSID and password to connect to network (data endpoints)
#define WIFI_SSID       "key_value"
#define WIFI_PASS       "key_value"

// Configuration Step 2: If using MQTT, set MQTT broker login parameters
#define MQTT_BROKER     "mqtt.hostname.local or IP address"
#define MQTT_PORT       port_number	// use 8883 for SSL
#define MQTT_USER       "key_value"
#define MQTT_PASSWORD   "key_value"

// Configuration Step 3: Set key device and installation configuration parameters.  These are used
// widely throughout the code to properly identify the device and generate important
// operating elements like MQTT topics, InfluxDB data tags (metadata).  Should be
// customized to match the target installation. Values here are examples.
#define DEVICE           "key_value"	// name of device, e.g. "realtime_co2"
#define DEVICE_SITE      "key_value"	// physical address of the device, e.g. "1234 Main Street"
#define DEVICE_LOCATION  "key_value"	// general location of device at physical address, e.g. "indoor"
#define DEVICE_ROOM      "key_value"	// specific location of device within location, e.g. "kitchen"
#define DEVICE_ID        "key_value"	// unique ID for the device, e.g. "007"

#define VALUE_KEY_LIGHT   "key_value" // 
#define VALUE_KEY_RSSI    "key_value" // 

// This line is required until a bug fix is completed. Copy values from config step 3.
#define MQTT_SUB_TOPIC    "DEVICE_SITE/DEVICE_LOCATION/DEVICE_ROOM/DEVICE/light"