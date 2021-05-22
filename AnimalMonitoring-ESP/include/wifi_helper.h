#define WIFI_H
#ifdef WIFI_H

#include<WiFi.h>
#include <AsyncMqttClient.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}




class WIFI_HELPER{
    public:
    AsyncMqttClient mqttClient;
    static void connectToWiFi(const char * ssid, const char * pwd);
    static void WiFiEvent(WiFiEvent_t event); 
    TimerHandle_t mqttReconnectTimer;
    TimerHandle_t wifiReconnectTimer;
    bool WIFI_connected;
    static void reconnect();
    
    
    
};


#endif