#include <PubSubClient.h>
#include <WiFi.h>

#include "mqtt.h"
#include "credentials.h"

extern int MPU_PKT_SIZE;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

boolean mqttInitCompleted = false;
//setting mac address of the ESP as the client ID
String clientId = "IoTPractice-" + String(WiFi.macAddress());

/* Incoming data callback. */
void dataCallback(char *topic, byte *payload, unsigned int length)
{
  char payloadStr[length + 1];
  memset(payloadStr, 0, length + 1);
  strncpy(payloadStr, (char *)payload, length);
  Serial.printf("Data    : dataCallback. Topic : [%s]\n", topic);
  Serial.printf("Data    : dataCallback. Payload : %s\n", payloadStr);
}

void performConnect()
{
  uint16_t connectionDelay = 5000;
  while (!mqttClient.connected())
  {
    Serial.printf("Trace   : Attempting MQTT connection...\n");
    //connect to the client and last will messages, just in case WIFI fails
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_KEY,Acceleration,0, true, "Disconnected from the server"))
    {
      Serial.printf("Trace   : Connected to Broker.\n");

      /* Subscription to your topic after connection was succeeded.*/
      MQTTSubscribe(Acceleration);
    }
    else
    {
      Serial.printf("Error!  : MQTT Connect failed, rc = %d\n", mqttClient.state());
      Serial.printf("Trace   : Trying again in %d msec.\n", connectionDelay);
      delay(connectionDelay);
    }
  }
}

boolean MQTTPublish(const char *topic, char *payload)
{
  boolean retval = false;
  if (mqttClient.connected())
  {
    retval = mqttClient.publish(topic, payload);
  }
  return retval;
}

boolean MQTTSubscribe(const char *topicToSubscribe)
{
  boolean retval = false;
  if (mqttClient.connected())
  {
    retval = mqttClient.subscribe(topicToSubscribe);
  }
  return retval;
}

boolean MQTTIsConnected()
{
  return mqttClient.connected();
}

void MQTTBegin()

{ //uncomment if you want to change the maximum packet size
  //mqttClient.setBufferSize(MPU_PKT_SIZE);

  //uncomment if you want to increase the Alive timme
  //mqttClient.setKeepAlive(KeepAlive);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(dataCallback);
  mqttInitCompleted = true;
}

void MQTTLoop()
{
  if (mqttInitCompleted)
  {
    if (!MQTTIsConnected())
    {
      performConnect();
    }
    mqttClient.loop();
  }
}