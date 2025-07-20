/*
  Project Name:   benchFace
  Description:    public (non-secret) configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 1

// Configuration variables that change rarely

// Network
// max connection attempts to network services
const uint8_t networkConnectAttemptLimit = 3;
// time between network service connect attempts
const uint8_t networkConnectAttemptInterval = 10; // seconds
// max time before MQTT connection is pinged to keep open
const uint16_t networkMQTTKeepAliveInterval = 300; // seconds

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS
const uint16_t sensorSampleInterval = 5; // seconds

#ifdef DEBUG
  const uint16_t faceDetectTimeoutWindow = 30; // seconds
#else
  const uint16_t faceDetectTimeoutWindow = 300; // seconds
#endif

// Hardware

// relay featherwing
const uint8_t hardwareRelayPin = 12;