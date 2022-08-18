
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "algorithms/Buffers.h"

#ifndef MPU6050_H
#define MPU6050_H

#define MPU 0x68

class MPU6050{
    public:
        MPU6050();
        void init(int);
        void read(HIDBuffer*);
        int id;

    private:
        unsigned long d_t;
        Adafruit_MPU6050  mpu;
        sensors_event_t a, g, temp;       //Declaring sensor event variables to be passed into the get event function that is included in the adafruit library 
};

#endif