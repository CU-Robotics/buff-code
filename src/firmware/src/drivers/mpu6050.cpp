// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>        //Including the librariy for the Adafruit 6050 IMU
#include <Wire.h>

#include "mpu6050.h"        //Including the header file for this driver

MPU6050::MPU6050() {
    id = -1;
    filterlvl = 0;
   
    gyro = Buffer3(10);
    accel = Buffer3(10);

    d_t = micros();

    //mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries
}

void MPU6050::init(int idx, int flvl){         //Our default constructor
    id = idx;
    filterlvl = flvl;

    // Serial.print("New imu "); Serial.print(id); Serial.print(" "); Serial.println(filterlvl);

    gyro = Buffer3(10);
    accel = Buffer3(10);

    d_t = millis();

    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries

}

void MPU6050::read(HIDBuffer* buffer){

    if (id == -1) {
        return;
    }

    if (millis() - d_t < 5){
        return;
    }

    if(!mpu.getEvent(&a, &g, &temp)){
        return;
    }

    // Serial.println("IMU packet");

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
        a_avg.x = a.acceleration.x;
        a_avg.y = a.acceleration.y;
        a_avg.z = a.acceleration.z;
    }
    

    if (!buffer->check_of(29)){
        buffer->put('T');
        buffer->put('T');
        buffer->put(24);
        buffer->put(id);
        buffer->put_f32(a_avg.x);
        buffer->put_f32(a_avg.y);
        buffer->put_f32(a_avg.z);
        buffer->put_f32(g_avg.x);
        buffer->put_f32(g_avg.y);
        buffer->put_f32(g_avg.z);
    }

    d_t = millis();
}
