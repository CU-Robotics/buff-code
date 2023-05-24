#include "icm20649.h"

ICM20649::ICM20649() {
    // start i2c communications
    icm20649.begin_I2C();
    
    // get the sensor objects linked to the sensor
    icm_temp = icm20649.getTemperatureSensor();
    icm_accel = icm20649.getAccelerometerSensor();
    icm_gyro = icm20649.getGyroSensor();
}

// void ICM20649::readSensor() {
//     icm_temp = icm20649.getTemperatureSensor();
//     icm_accel = icm20649.getAccelerometerSensor();
//     icm_gyro = icm20649.getGyroSensor();
// }

void ICM20649::readSensor() {
    icm_temp->getEvent(&temp);
    icm_accel->getEvent(&accel);
    icm_gyro->getEvent(&gyro);

    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;

    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;
}

void ICM20649::read_accel() {
    icm_accel->getEvent(&accel);
    data[0] = accel.acceleration.x;
    data[1] = accel.acceleration.y;
    data[2] = accel.acceleration.z;
}

void ICM20649::read_gyro() {
    icm_gyro->getEvent(&gyro);
    data[3] = gyro.gyro.x;
    data[4] = gyro.gyro.y;
    data[5] = gyro.gyro.z;
}
