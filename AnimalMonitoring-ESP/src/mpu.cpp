#include "mpu.h"
#include "util.h"
#include "main.h"
#include "credentials.h"
#include "wifi_helper.h"

#include <Wire.h>
#include <EEPROM.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// ================== USER CONFIG ==================

// Module ID
#define MODULE_ID "AFM_2"

// Networking setup
//#define COMPUTER_WIRED
#define COMPUTER_WIFI
//#define PUPSNET

// Which sensors used?
#define USE_MPU

//int ledPin = 2;

//The udp library class
WiFiUDP udp;
//WiFiServer tcpServer(3335);

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// Set initial input parameters
enum Ascale
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
int16_t accelCount[3];
int Ascale = AFS_2G;
float aRes;
float ax, ay, az;

// ==================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

typedef struct
{

#ifdef OUTPUT_READABLE_QUATERNION
  Quaternion q;
#endif

#ifdef OUTPUT_READABLE_EULER
  float euler[3];
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  float ypr[3];
#endif

#ifdef OUTPUT_READABLE_RAWACCEL
  VectorInt16 aa;
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  VectorInt16 aaReal;
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
  VectorInt16 aaWorld;
#endif

#ifdef OUTPUT_FIFOBUFFER
  int16_t fifoBuffer[7];
#endif
} MPU_PKT;

MPU_PKT mpu_pkt;
int MPU_PKT_SIZE = sizeof(MPU_PKT);
//#define MPU_PKT_SIZE sizeof(MPU_PKT)

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#ifdef OUTPUT_TEAPOT
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
#endif

// ==================================================

static int i2cCore = 1;

int16_t offsets[6];

bool mpuDataReady = false;
bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

//int mpuDataCounter = 0;
//int mpuDataCounterPrev = 0;

bool blinkState = false;

// ================================================================
// ===                INTERRUPT SERVICE ROUTINES                ===
// ================================================================

void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

void MPU::resetOffsets()
{
  // Get the saved offsets from the EEPROM
  EEPROM.get(1, offsets);

  // Calibrate using known offsets
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);

  // Show the current offsets
  mpu.PrintActiveOffsets();
}

void getAres()
{
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
  case AFS_2G:
    aRes = 2.0 / 32768.0;
    break;
  case AFS_4G:
    aRes = 4.0 / 32768.0;
    break;
  case AFS_8G:
    aRes = 8.0 / 32768.0;
    break;
  case AFS_16G:
    aRes = 16.0 / 32768.0;
    break;
  }
}

void MPU::get_core()
{
  delay(100);

  String taskMessage = "sensorTask running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  Serial.println(taskMessage);
}

void startDMP()
{

#ifdef USE_MPU
  // load and configure the DMP
  mpu.reset();
  delay(30);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {

    MPU::resetOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);

#endif // USE_MPU
}

void MPU::__init__()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // Wire.setClock(100000); // 100kHz I2C clock. Comment this line if having compilation difficulties

  delay(1000);

#ifdef USE_MPU

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  delay(100);

  mpu.reset(); //help startup reliably - doesn't always work though.
  // maybe can also reset i2c on esp32?

  delay(100);

  mpu.resetI2CMaster();

  delay(100);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN_MPU, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// Here the calibration of mpu sensor happens and the offsets are stored in the EEPROM
// This process is required to be done only once
#ifdef CALIBRATE_MPU
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel();
  mpu.CalibrateGyro();

  // Get current offsets
  mpu.PrintActiveOffsets(offsets);

  // Save the offsets in the EEPROM
  EEPROM.put(1, offsets);
#endif

  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again
  // LowPowerMode();
  startDMP();
#endif
}

void MPU::printPkt()
{
#ifdef OUTPUT_READABLE_QUATERNION
  // display quaternion values in easy matrix form: w x y z
  Serial.print("quat\t");
  Serial.print(mpu_pkt.q.w);
  Serial.print("\t");
  Serial.print(mpu_pkt.q.x);
  Serial.print("\t");
  Serial.print(mpu_pkt.q.y);
  Serial.print("\t");
  Serial.println(mpu_pkt.q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
  // display Euler angles in degrees
  Serial.print("euler\t");
  Serial.print(mpu_pkt.euler[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(mpu_pkt.euler[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(mpu_pkt.euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  // display Euler angles in degrees
  Serial.print("ypr\t");
  Serial.print(mpu_pkt.ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(mpu_pkt.ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(mpu_pkt.ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_RAWACCEL
  // display raw acceleration, adjusted to remove gravity
  Serial.print("arawl\t");
  Serial.print(mpu_pkt.aa.x);
  Serial.print("\t");
  Serial.print(mpu_pkt.aa.y);
  Serial.print("\t");
  Serial.println(mpu_pkt.aa.z);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  // display real acceleration, adjusted to remove gravity
  Serial.print("areal\t");
  Serial.print(mpu_pkt.aaReal.x);
  Serial.print("\t");
  Serial.print(mpu_pkt.aaReal.y);
  Serial.print("\t");
  Serial.println(mpu_pkt.aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
  Serial.print("aworld\t");
  Serial.print(mpu_pkt.aaWorld.x);
  Serial.print("\t");
  Serial.print(mpu_pkt.aaWorld.y);
  Serial.print("\t");
  Serial.println(mpu_pkt.aaWorld.z);
#endif
}

void MPU::processPkt()
{
// Actual quaternion components in a [w, x, y, z] format (not best
// for parsing on a remote host such as Processing or something though)
#if defined(OUTPUT_READABLE_QUATERNION)
  Quaternion q; // [w, x, y, z]         quaternion container
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  memcpy(&(mpu_pkt.q), &q, sizeof(Quaternion));
#endif

// Euler angles (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#if defined(OUTPUT_READABLE_EULER)
  Quaternion q;   // [w, x, y, z]         quaternion container
  float euler[3]; // [psi, theta, phi]    Euler angle container
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  memcpy(mpu_pkt.euler, euler, sizeof(float) * 3);
#endif

// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
// Note that Yaw, Pitch, and Roll and angles in degress that are calculated by
// accumulating gyroscope readings (angular velocities).
#if defined(OUTPUT_READABLE_YAWPITCHROLL)
  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorFloat gravity; // [x, y, z]            gravity vector
  float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  memcpy(mpu_pkt.ypr, ypr, sizeof(float) * 3);
#endif

#if defined(OUTPUT_READABLE_RAWACCEL)
  VectorInt16 aa;                   // [x, y, z]            accel sensor measurements
  mpu.dmpGetAccel(&aa, fifoBuffer); // Real Acceleration
  memcpy(&(mpu_pkt.aa), &aa, sizeof(VectorInt16));
#endif

// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, use OUTPUT_READABLE_WORLDACCEL instead.
#if defined(OUTPUT_READABLE_REALACCEL)
  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorInt16 aa;      // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
  VectorFloat gravity; // [x, y, z]            gravity vector
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aa, fifoBuffer);              // Real Acceleration
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // Acceleration components with gravity removed
  memcpy(&(mpu_pkt.aaReal), &aaReal, sizeof(VectorInt16));
#endif

// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#if defined(OUTPUT_READABLE_WORLDACCEL)
  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorInt16 aa;      // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity; // [x, y, z]            gravity vector
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aa, fifoBuffer);                    // Real Acceleration
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);       // Acceleration components with gravity removed
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q); // Acceleration components with gravity removed and converted back to world frame
  memcpy(&(mpu_pkt.aaWorld), &aaWorld, sizeof(VectorInt16));
#endif

// Special format for extraction of minimum data from FIFO Buffer and shifting
// all the processing to the gateway or cloud
#ifdef OUTPUT_FIFOBUFFER
  mpu.dmpGetQuaternion(mpu_pkt.fifoBuffer, fifoBuffer);
  mpu.dmpGetAccel(&(mpu_pkt.fifoBuffer[4]), fifoBuffer);
#endif

#ifdef IMU_SERIAL_DEBUG
  printPkt();
#endif
}

uint8_t *MPU::readPkt()
{
  // Depends if the DMP is turned on or off right now!
  if (!dmpReady)
  {
    if (mpuInterrupt)
    {
      // This interrupt here tells us that the system woke up from zero motion.
      mpu.getAcceleration(&accelCount[0], &accelCount[1], &accelCount[2]);
      getAres();

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
      ay = (float)accelCount[1] * aRes;
      az = (float)accelCount[2] * aRes;

      Serial.print("ax = ");
      Serial.print((int)1000 * ax);
      Serial.print(" ay = ");
      Serial.print((int)1000 * ay);
      Serial.print(" az = ");
      Serial.print((int)1000 * az);
      Serial.println(" mg");
      mpuInterrupt = false;

      // sleep_disable();
      detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU)); // Detach whatever interrupt routing low power mode was using
      startDMP();                                                // This is the point where you go back to the full mode of readings.
    }
    else
    {
      //sleep_cpu();
    }
  }
  else
  {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
// Special format for visualzing MPU readings in "Processing" software
#ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:

      uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'}; // packet structure for InvenSense teapot demo
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
      Serial.write(teapotPacket, 14);
#else
      processPkt();
      return (uint8_t *)&mpu_pkt.aaReal.x;
      Serial.print(mpu_pkt.aaReal.x);

      //new return type

#endif
    }
  }
  return NULL;
}

// ================================================================
// ===              I2C SENSOR READ TASK (CORE 0)               ===
// ================================================================

/*
void MPU::sensorTask( void * pvParameters ) {

  // ================== SETUP ==================

  MPU::get_core();

  // join I2C bus (I2Cdev library doesn't do this automatically)

 MPU::__init__();




  // ================== LOOP ==================

  while (true) {
    if (WIFI_connected) {

#ifdef USE_MPU

      // if programming failed, don't try to do anything
      if (!dmpReady) {
        Serial.println("dmpNotReady");
        delay(100);
      }

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
         //
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        

        

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        MPU::readPkt();
        MPU::printPkt();



        mpuDataCounter++;

        // blink LED to indicate activity - need to move this elsewhere
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }

#endif // USE_MPU

    } else { // not connected
      vTaskDelay(10); // wait and feed the watchdog timer
    }
  } // end of loop
} // end sensorTask
*/

/*
void setup() {
  

  // initialize serial communication

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  // configure LED for output
  pinMode(ledPin, OUTPUT);

  //Connect to the WiFi network
  WIFI_HELPER::connectToWiFi(WIFI_SSID,WIFI_PASSWORD);

  delay(100);

  Serial.print("Creating i2c task on core ");
  Serial.println(i2cCore);
  xTaskCreatePinnedToCore(
    MPU::sensorTask,   // Function to implement the task 
    "coreTask", // Name of the task 
    10000,      //Stack size in words 
    NULL,       // Task input parameter 
    20,          // Priority of the task 
    NULL,       // Task handle. 
    i2cCore);  // Core where the task should run 

  Serial.println("i2c task created.");
} */

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() {
  if (WIFI_connected) {
    // UDP

    // improve to only xmit when changed
    // either compare to last, or keep track of when new data has been read
    // when new data increment a global counter. check here if different from last read
    // if so, send packet and update previousCounter val.
    // use separate counters for each sensor.

#ifdef USE_MPU
    if (mpuDataCounter != mpuDataCounterPrev) {
      // format: MODULE_ID mpu yaw pitch roll wx wy wz rx ry rz gravx gravy gravz
      
      Serial.print(MODULE_ID);
      Serial.print(" mpu ");
      

      //      udp.print(" areal ");
      Serial.print(aaReal.x);
      Serial.print(" ");
      Serial.print(aaReal.y);
      Serial.print(" ");
      Serial.println(aaReal.z);

      Serial.print(" ");
      Serial.print(gravity.x);
      Serial.print(" ");
      Serial.print(gravity.y);
      Serial.print(" ");
      Serial.print(gravity.z);

      delay(3000);
    

      mpuDataCounterPrev = mpuDataCounter;
    }

#endif // USE_MPU

  } else { // not connected
    vTaskDelay(10); // wait and feed the watchdog timer.
  }
}
*/