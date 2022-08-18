// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>        //Including the librariy for the Adafruit 6050 IMU
#include <Wire.h>

#include "mpu6050.h"        //Including the header file for this driver

MPU6050::MPU6050() {
    id = -1;
    d_t = micros();
    //mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries
}

void MPU6050::init(int idx){         //Our default constructor
    id = idx;
    Serial.print("New imu "); Serial.println(id);
    d_t = millis();
    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries

}

void MPU6050::read(HIDBuffer* buffer){

    if (millis() - d_t < 10){
        return;
    }

    if(!mpu.getEvent(&a, &g, &temp)){
        return;
    }

    // Serial.println("IMU packet");

    if (!buffer->check_of(29)){
        buffer->put('T');
        buffer->put('T');
        buffer->put(24);
        buffer->put(id);
        buffer->put_f32(a.acceleration.x);
        buffer->put_f32(a.acceleration.y);
        buffer->put_f32(a.acceleration.z);
        buffer->put_f32(g.gyro.x);
        buffer->put_f32(g.gyro.y);
        buffer->put_f32(g.gyro.z);
    }

    d_t = millis();
}
