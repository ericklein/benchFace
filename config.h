/*
  Project Name:   benchFace
  Description:    public (non-secret) configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 2

// Configuration variables that change rarely

// Network timers
const uint32_t timeMQTTKeepAliveIntervalMS = 300000; // ping MQTT broker every 300 seconds to keep alive
const uint32_t timeWiFiKeepAliveIntervalMS = 30000; // Retry every 30 seconds
const uint32_t timeNetworkConnectTimeoutMS = 10000;

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS
const uint32_t sensorSampleIntervalMS = 5000;

#ifdef DEBUG
  const uint32_t faceDetectTimeoutWindowMS = 30000;
#else
  const uint32_t faceDetectTimeoutWindowMS = 300000;
#endif

// Hardware

// relay featherwing
const uint8_t hardwareRelayPin = 12;