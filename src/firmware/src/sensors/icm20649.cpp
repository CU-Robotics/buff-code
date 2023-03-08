#include "icm20649.h"

icm20649::icm20649() {
    // start i2c communications
    icm.begin_I2C();
    
    // get the sensor objects linked to the sensor
    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAcellerometerSensor();
    icm_gyro = icm.getGyroSensor();
}

void icm20649::readSensor() {
    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAcellerometerSensor();
    icm_gyro = icm.getGyroSensor();
}

void icm20649::readSensor() {
    icm_temp->getEvent(&temp);
    icm_accel->getEvent(&accel);
    icm_gyro->getEvent(&gyro);

    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;

    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.Y;
    gyroZ = gyro.gyro.Z;
}
