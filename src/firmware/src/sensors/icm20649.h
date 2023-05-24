// #include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
// #include <Adafruit_Sensor.h>

#ifndef BUFF_ICM20649_H
#define BUFF_ICM20649_H

#define ICM20649_DOF 6

class ICM20649 {
public:
    ICM20649();
    void readSensor();
    void read_accel();
    void read_gyro();
    float getTemperature(){ return temp.temperature; };
    
    float getAccelX() { return accelX; };
    float getAccelY() { return accelY; };
    float getAccelZ() { return accelZ; };

    float getGyroX() { return gyroX; };
    float getGyroY() { return gyroY; };
    float getGyroZ() { return gyroZ; };
    float data[ICM20649_DOF];


private:
    Adafruit_ICM20649 icm20649;
    Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;


    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
};

#endif
