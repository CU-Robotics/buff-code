#include <Adafruit_ICM20649>

#ifndef BUFF_ICM20649_H
#define BUFF_ICM20649_H

class ICM20649 {
public:
    ICM20649();
    void readSensor();
    float getTemperature(){ return temp.temperature; };
    
    float getAccelX() { return accelX; };
    float getAccelY() { return accelY; };
    float getAccelZ() { return accelZ; };

    float getGyroX() { return gyroX; };
    float getGyroY() { return gyroY; };
    float getGyroZ() { return gyroZ; };
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
