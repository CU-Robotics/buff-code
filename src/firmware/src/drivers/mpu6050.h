
#include <Wire.h>
#include <Adafruit_MPU6050.h>

#include "algorithms/Buffers.h"

#ifndef MPU6050_H
#define MPU6050_H

#define MPU 0x68

class MPU6050{
    public:
        MPU6050();
        void init(int, int);
        void read(HIDBuffer*);

    private:
        int id;
        int filterlvl;
        unsigned long d_t;
        Buffer3 gyro;
        Buffer3 accel;
        Adafruit_MPU6050  mpu;
        sensors_event_t a, g, temp;       //Declaring sensor event variables to be passed into the get event function that is included in the adafruit library 
};

#endif