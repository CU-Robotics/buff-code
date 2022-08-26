// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>        //Including the librariy for the Adafruit 6050 IMU
#include <Wire.h>

#include "mpu6050.h"        //Including the header file for this driver

typedef union
{
  float number;
  byte bytes[4];
} FLOATUNION_t;

MPU6050::MPU6050() {
    d_t = micros();
    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries
}


bool MPU6050::read(byte* buffer){

    // if (millis() - d_t < 10){
    //     return;
    // }

    if(!mpu.getEvent(&a, &g, &temp)){
        return false;
    }

    int offset = 0;
    FLOATUNION_t fu;
    fu.number = a.acceleration.x;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    offset += 1;
    fu.number = a.acceleration.y;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    offset += 1;
    fu.number = a.acceleration.z;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    offset += 1;
    fu.number = g.gyro.x;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    offset += 1;
    fu.number = g.gyro.y;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    offset += 1;
    fu.number = g.gyro.z;
    for (int i = 0; i < 4; i++){
        buffer[i+offset] = fu.bytes[i];
    }

    d_t = millis();
    return true;
}
