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

    //setupt motion detection
    // mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    // mpu.setMotionDetectionThreshold(1);
    // mpu.setMotionDetectionDuration(20);
    // mpu.setInterruptPinLatch(true);                         // Keep it latched.  Will turn off when reinitialized.
    // mpu.setInterruptPinPolarity(true);
    // mpu.setMotionInterrupt(true);
}

void MPU6050::init(int idx, int flvl){         //Our default constructor
    id = idx;
    filterlvl = flvl;
   
    gyro = Buffer3(10);
    accel = Buffer3(10);

    d_t = millis();

    mpu.begin();                                             //Calling the mpu begin function that is included in the Adafruit libraries

    //setupt motion detection
    //mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    //mpu.setCycleRate(MPU6050_CYCLE_40_HZ);

    // Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
    // Wire.write(0x6B);                  // Talk to the register 6B
    // Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    // Wire.endTransmission(true);        //end the transmission
}

void MPU6050::read(HIDBuffer* buffer){

    if (id == -1) {
        return;
    }

    if (millis() - d_t < 25){
        return;
    }

    unsigned long t = micros();
    unsigned long t1 = micros();
    bool success = mpu.getEvent(&a, &g, &temp);           //calling the getEvent function which updates the sensor events passed through its parameters

    // Wire.beginTransmission(MPU);
    // Serial.print("transmition Time: ");
    // Serial.println(micros() - t);
    // t1 = micros();
    // Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    // Serial.print("wire write Time: ");
    // Serial.println(micros() - t1);
    // t1 = micros();
    // Wire.endTransmission(false);
    // Serial.print("End Time: ");
    // Serial.println(micros() - t1);
    // t1 = micros();
    // Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    // Serial.print("wire request Time: ");
    // Serial.println(micros() - t1);
    // //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    // float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    // float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    // float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    // Serial.print(" half getEvent Time: ");
    // Serial.println(micros() - t);

    // Wire.beginTransmission(MPU);
    // Wire.write(0x43); // Gyro data first register address 0x43
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    // float GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    // float GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    // float GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    

    // gyro.push(GyroX, GyroY, GyroZ);
    // accel.push(AccX, AccY, AccZ);

    gyro.push(g.gyro.x, g.gyro.y, g.gyro.z);
    accel.push(a.acceleration.x, a.acceleration.y, a.acceleration.z);

    Vector3 a_avg;
    Vector3 g_avg;
    if (filterlvl > 0) {
        g_avg = gyro.mean();
        a_avg = accel.mean();
    }
    else {
        // g_avg.x = GyroX;
        // g_avg.y = GyroY;
        // g_avg.z = GyroZ;
        // a_avg.x = AccX;
        // a_avg.y = AccY;
        // a_avg.z = AccZ;
        g_avg.x = g.gyro.x;
        g_avg.y = g.gyro.y;
        g_avg.z = g.gyro.z;
        a_avg.x = a.acceleration.x;
        a_avg.y = a.acceleration.y;
        a_avg.z = a.acceleration.z;
    }
    

    if (!buffer->check_of(29)){
        buffer->put('X');
        buffer->put('X');
        buffer->put(24);
        buffer->put(id);
        buffer->put(2);
        buffer->put_f32(a_avg.x);
        buffer->put_f32(a_avg.y);
        buffer->put_f32(a_avg.z);
        buffer->put_f32(g_avg.x);
        buffer->put_f32(g_avg.y);
        buffer->put_f32(g_avg.z);
    }

    d_t = millis();
}
