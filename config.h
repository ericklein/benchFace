/*
  Project Name:   benchFace
  Description:    public (non-secret) configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 2

// Configuration variables that change rarely

// timers
const uint32_t timeMQTTKeepAliveIntervalMS = 300000; // ping MQTT broker every 300 seconds to keep alive
// const uint32_t timeWiFiKeepAliveIntervalMS = 30000; // Retry every 30 seconds
const uint32_t timeNetworkConnectTimeoutMS = 10000; // used by network endpoint code

const uint8_t networkConnectAttemptLimit = 3; // used by network endpoint code

// sample rates
// How long to wait people sensor reads. Results only change ~ 5FPS
const uint32_t sensorSampleIntervalMS = 2000;
const uint16_t mqttSubSampleIntervalMS = 2000;

#ifdef DEBUG
  const uint32_t faceDetectTimeoutWindowMS = 30000;
#else
  const uint32_t faceDetectTimeoutWindowMS = 300000;
#endif

// hardware
const String hardwareDeviceType = "benchLight";

// relay featherwing
const uint8_t hardwareRelayPin = 12;

// button
const uint8_t hardwareWipeButton = 38; // second button on Adafruit Feather ESP32V2 board
const uint16_t timeResetButtonHoldMS = 10000; // Long-press duration to wipe config