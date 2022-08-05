
#include <Adafruit_MPU6050.h>

#include "algorithms/Buffers.h"

#ifndef MPU6050_H
#define MPU6050_H

class MPU6050{
    public:
        MPU6050();
        MPU6050(int, int);
        void read(HIDBuffer*);

    private:
        int hidid;
        int filterlvl;
        Buffer3 gyro;
        Buffer3 accel;
        Adafruit_MPU6050  mpu;
        sensors_event_t a, g, temp;       //Declaring sensor event variables to be passed into the get event function that is included in the adafruit library 
};

#endif