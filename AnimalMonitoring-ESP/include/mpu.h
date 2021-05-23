#ifndef MPU_H
#define MPU_H

#include "main.h"
#include "Arduino.h"
#include <WiFi.h>


class MPU
{
    public:
    static void __init__();
    static void sensorTask( void * pvParameters );
    static void get_core();
    static bool start();
    static char* readPkt();
    static void processPkt();
    static void printPkt();
    static void resetOffsets();
    static void MPUreset();
};



#endif