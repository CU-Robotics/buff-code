
#include <Wire.h>
#include <Adafruit_MPU6050.h>

#ifndef MPU6050_H
#define MPU6050_H

class MPU6050{
    public:
        MPU6050();
        bool read(byte*);

    private:
        unsigned long d_t;
        Adafruit_MPU6050  mpu;
        sensors_event_t a, g, temp;       //Declaring sensor event variables to be passed into the get event function that is included in the adafruit library 
};

#endif