#include "credentials.h"
#include "wifi_helper.h"
#include "main.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncMqttClient.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}



// TO ADD
// increase alive time

//initializing the MQTT client
//AsyncMqttClient mqttClient;

//TimerHandle_t mqttReconnectTimer;
//TimerHandle_t wifiReconnectTimer;

WIFI_HELPER w;

bool WIFI_connected = false;
//bool MQTT_session = false;

//wifi event handler
void WIFI_HELPER::WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    // When connected set
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());

    //connect to the MQTT SERVER
    //WIFI_HELPER::connectToMqtt();

    Serial.printf("Wifi Event running on core %d", (int)xPortGetCoreID());
    Serial.println();

    w.WIFI_connected = true;

    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    w.WIFI_connected = false;
    WIFI_HELPER::reconnect();
    xTimerStart(w.wifiReconnectTimer, 0);
    break;

  case SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP:
    break;

  case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    break;

  case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
    break;

  case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
    break;

  case SYSTEM_EVENT_AP_STADISCONNECTED:
    break;
  case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    break;
  case SYSTEM_EVENT_AP_PROBEREQRECVED:
    break;
  case SYSTEM_EVENT_AP_STAIPASSIGNED:
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    break;
  case SYSTEM_EVENT_STA_WPS_ER_PIN:
    break;
  case SYSTEM_EVENT_STA_CONNECTED:
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    break;
  case SYSTEM_EVENT_STA_LOST_IP:
    break;

  default:
    break;
  }
}

//function to connect to wifi and the MQTT server
void WIFI_HELPER::connectToWiFi(const char *ssid, const char *pwd)
{
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WIFI_HELPER::WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

void WIFI_HELPER::reconnect(){
  Serial.print("Reconnecting to WIFI.................");
  WiFi.reconnect();
  w.WIFI_connected = true;
  
}