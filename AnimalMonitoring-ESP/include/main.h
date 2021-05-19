#ifndef MAIN_H
#define MAIN_H

// 1. Uncomment "SET_ID" when you want to program the transmitter id explicitly
// 2. Uncomment "CALIBRATE_MPU" when you want to initialize the offsets of the 
      // MPU unit
// 3. Uncomment "IMU_SERIAL_DEBUG" when you are using any special computation on 
      // gyro or accel readings and want to see the processed readings.

/*
 * Basic configurations
 */
#define SERIAL_BAUD_RATE 115200
// #define SET_ID 3 
#define N_NODES 4


/*
 * MPU configurations
 */
#define INTERRUPT_PIN_MPU 19
// #define CALIBRATE_MPU
#define SAMPLE_RATE_DIVIDER 4
#define IMU_SERIAL_DEBUG
#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

/*
 * Uncomment if you want any special computation on gyroscope readings 
 */
// #define OUTPUT_READABLE_QUATERNION
// #define OUTPUT_READABLE_EULER
// #define OUTPUT_READABLE_YAWPITCHROLL

/*
 * Uncomment if you want any special computation on accelerometer readings
 */
// #define OUTPUT_READABLE_RAWACCEL
#define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL

/*
 * Uncomment "OUTPUT_FIFOBUFFER" if you want just the raw buffer values
 */
// #define OUTPUT_FIFOBUFFER

/*
 * Uncomment "OUTPUT_TEAPOT" if you want output that matches the format used for the InvenSense teapot demo
 */
//#define OUTPUT_TEAPOT

//to check if MQTT server is responding
//#define MQTT_TEST


#define MQTT_PKT_SIZE 10 // Number of measurement packets to pack in 1 LoRa packet
#define Async_MQTT


/* 
 * Piezo
 */
#define PIEZO_PIN 3 // the analog pin connected to the sensor
#define PIEZO_THRESHOLD 100

#endif
