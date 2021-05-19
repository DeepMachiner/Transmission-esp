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
    xTimerStop(w.mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
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

/*
void WIFI_HELPER::connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  MQTT.mqttClient.connect();
  if(MQTT.mqttClient.connected()){
    Serial.println("Connected to MQTT server");
    MQTT_session = true;
  }
}

void WIFI_HELPER::onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  //MQTT connected
  //MQTT_session = true;

  Serial.println(sessionPresent);

//to test if the MQTT server is responding
#ifdef MQTT_TEST
  MQTT.mqttClient.publish(Acceleration, 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = MQTT.mqttClient.publish(Acceleration, 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = MQTT.mqttClient.publish(Gyroscope, 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
#endif
}

void WIFI_HELPER::onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  //Serial.println(reason);
  MQTT_session = false;
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void WIFI_HELPER::onMqttPublish(uint16_t packetId)
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

WIFI_HELPER WIFI_HELPER::mqtt__init__()
{
  //connect to wifi
  WIFI_HELPER::connectToWiFi(WIFI_SSID, WIFI_PASSWORD);

  
  //event handles for MQTT functions
  MQTT.mqttClient.onConnect(WIFI_HELPER::onMqttConnect);
  MQTT.mqttClient.onDisconnect(WIFI_HELPER::onMqttDisconnect);
  MQTT.mqttClient.onPublish(WIFI_HELPER::onMqttPublish);

  // credentials for mqtt broker
  MQTT.mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  MQTT.mqttClient.setCredentials(MQTT_USERNAME, MQTT_KEY);
    
}*/
