#ifndef benchFace_H
  #define benchFace_H

  #include <Arduino.h>  // for String, uint16_t

  struct MqttConfig{
    String host;
    uint16_t port;
    String user;
    String password;
  } ;
  extern MqttConfig mqttBrokerConfig;

  struct networkEndpointConfig{
    String site;
    String location;
    String room;
    String deviceID;
  };
  extern networkEndpointConfig endpointPath;
#endif