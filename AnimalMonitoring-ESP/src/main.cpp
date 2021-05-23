#include "mpu.h"
#include "util.h"
#include "main.h"
#include "credentials.h"
#include "wifi_helper.h"
#include "mqtt.h"


#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include<time.h>

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
const long interval = 1000;       // Interval at which to publish sensor readings

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
char *MQTT_pkt = (char *)malloc(MQTT_PKT_SIZE * MPU_PKT_SIZE);
int emptycounter;
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
 
 

  if (w.WIFI_connected)
  {

  MQTTLoop();
  char *mpu_pkt = MPU::readPkt();
  currentMillis = millis();

    
 
    // Build the MQTT pkt and send if full
   if (mpu_pkt)
        {
          //reset every time a packet is sent
          emptycounter = 0;

      
     //packet format nodeID,Ax,Ay,Az
     sprintf(MQTT_pkt,",%d,%s",nodeID,mpu_pkt);
      MQTT_pkt_idx += 1;
      MQTT_pkt_idx %= MQTT_PKT_SIZE;

     if (!MQTT_pkt_idx)
      {
        
        //if (currentMillis - previousMillis >= interval)
        //{ 
          SerialDebugger(F("Sending pkt number "), MQTT_pkt_counter++, F(" to the server"));
          Serial.print(MODULE_ID);
          Serial.print(" mpu ");
          
                 
          
          // Save the last time a new reading was published
          previousMillis = currentMillis;

          if(MQTTPublish(Acceleration, MQTT_pkt)){
            Serial.println("Published to Server");
          }
            
          mpuDataCounter++;
      //  }

        mpuDataCounterPrev = mpuDataCounter;
     }
   }
    else
    {      //packet is empty
           Serial.print("Packet Empty"); 
           emptycounter++;
           delay(5000);
           // if more than x number of empty packets from mpu, reset MPU or restart the board
           if(emptycounter > 5)
           {
             MQTTPublish(Acceleration, "Error code 1,resetting board");
           MPU::MPUreset();
           }
               
    }
  }
  else
  WIFI_HELPER::reconnect();

}
