// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>        //Including the librariy for the Adafruit 6050 IMU
// #include <Wire.h>

#include "mpu6050.h"        //Including the header file for this driver

MPU6050::MPU6050() {
    hidid = -1;
    filterlvl = 0;
   
    gyro = Buffer3(10);
    accel = Buffer3(10);

    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);                         // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
}

MPU6050::MPU6050(int id, int flvl){         //Our default constructor
    hidid = id;
    filterlvl = flvl;
   
    gyro = Buffer3(10);
    accel = Buffer3(10);

    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);                         // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
}

void MPU6050::read(HIDBuffer* buffer){

    bool success = mpu.getEvent(&a, &g, &temp);           //calling the getEvent function which updates the sensor events passed through its parameters

    if (success && hidid != -1) {
        gyro.push(g.gyro.x, g.gyro.y, g.gyro.z);
        accel.push(a.acceleration.x, a.acceleration.y, a.acceleration.z);

        Vector3 a_avg;
        Vector3 g_avg;
        if (filterlvl > 0) {
            g_avg = gyro.mean();
            a_avg = accel.mean();
        }
        else {
            g_avg.x = g.gyro.x;
            g_avg.y = g.gyro.y;
            g_avg.z = g.gyro.z;
            a_avg.x = a.gyro.x;
            a_avg.y = a.gyro.y;
            a_avg.z = a.gyro.z;
        }
        

        if (!buffer->check_of(27)){
            buffer->put('X');
            buffer->put('X');
            buffer->put(hidid);
            buffer->put(24);
            buffer->put_f32(a_avg.x);
            buffer->put_f32(a_avg.y);
            buffer->put_f32(a_avg.z);
            buffer->put_f32(g_avg.x);
            buffer->put_f32(g_avg.y);
            buffer->put_f32(g_avg.z);
        }
    } 
}
