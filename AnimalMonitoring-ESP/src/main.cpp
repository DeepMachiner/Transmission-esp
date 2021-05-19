#include "mpu.h"
#include "util.h"
#include "main.h"
#include "credentials.h"
#include "wifi_helper.h"
#include "mqtt.h"


#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <PubSubClient.h>




//node id
uint8_t nodeID = 0;

//WIFI and MQTT session
extern bool WIFI_connected;
extern bool MQTT_session;

//timer variables
unsigned long currentMillis;
unsigned long previousMillis = 0; // Stores last time packet was published
const long interval = 3000;       // Interval at which to publish sensor readings

//MQTT and WIFI
extern TimerHandle_t mqttReconnectTimer;
extern TimerHandle_t wifiReconnectTimer;
extern WIFI_HELPER w;
extern AsyncMqttClient mqttClient;
int MQTT_pkt_idx = 0;
int MQTT_pkt_counter = 0;


//core we're using
static int i2cCore = 1;

// Module ID
#define MODULE_ID "AFM_2"

// MPU
extern int MPU_PKT_SIZE; // Size of the packet from MPU6050 in bytes
uint8_t *MQTT_pkt = (uint8_t *)malloc(MQTT_PKT_SIZE * MPU_PKT_SIZE);
// Which sensors used?
#define USE_MPU

int mpuDataCounter = 0;
int mpuDataCounterPrev = 0;


void setup()
{

  Serial.begin(SERIAL_BAUD_RATE);

#ifdef SET_ID
  uint8_t nodeId = SET_ID;

  SerialDebugger(F("Setting nodeId..."));

  EEPROM.write(0, nodeId);
  SerialDebugger(F("Set nodeId = "), nodeId);
#endif

  nodeID = EEPROM.read(0);

  //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(WIFI_HELPER::connectToMqtt));
  //wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(WIFI_HELPER::connectToWiFi));

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //Connect to the WiFi network
  WIFI_HELPER::connectToWiFi(WIFI_SSID,WIFI_PASSWORD);

  //initialize MQTT Credentials and event handlers
  MQTTBegin();
  

  //initialize MPU
  MPU::__init__();
  

  delay(100);
  
}



void loop()
{
 
  //MQTTPublish(Acceleration, "test maqiatto");

  if (w.WIFI_connected)
  {

  MQTTLoop();
  uint8_t *mpu_pkt = MPU::readPkt();
  //VectorInt16 a = mpu_pkt.aaAreal.x;
  currentMillis = millis();
  Serial.print(MODULE_ID);
  Serial.print(" mpu ");
    
    // improve to only xmit when changed
    // either compare to last, or keep track of when new data has been read
    // when new data increment a global counter. check here if different from last read
    // if so, send packet and update previousCounter val.
    // use separate counters for each sensor.

    // format: MODULE_ID mpu yaw pitch roll wx wy wz rx ry rz gravx gravy gravz
    
    // Build the MQTT pkt and send if full
    if (mpu_pkt)
        {

      
      memcpy((void *)(MQTT_pkt + MQTT_pkt_idx * MPU_PKT_SIZE), mpu_pkt, MPU_PKT_SIZE);
      MQTT_pkt_idx += 1;
      MQTT_pkt_idx %= MQTT_PKT_SIZE;

      if (!MQTT_pkt_idx)
      {
        
        if (currentMillis - previousMillis >= interval)
        { 
          SerialDebugger(F("Sending pkt number "), MQTT_pkt_counter++, F(" to the server"));
          String packet = (char*)MQTT_pkt;
          //packet = packet.c_str();
          Serial.println("Printing Packet : " + packet);
          
          // Save the last time a new reading was published
          previousMillis = currentMillis;

          if(MQTTPublish(Acceleration, (char*)MQTT_pkt)){
            Serial.println("Published to Server");
          }
            
          mpuDataCounter++;
        }

        mpuDataCounterPrev = mpuDataCounter;
      }
    }
    else
    {       
               // not connected
      //vTaskDelay(10); // wait and feed the watchdog timer.
    }
  }
  else
  Serial.println("Not Connected to Wifi");


}
