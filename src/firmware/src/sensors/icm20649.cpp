#include "icm20649.h"
#include "buff_cpp/timing.h"

ICM20649::ICM20649() {
    // icm20649 = NULL;
}

// ICM20649::ICM20649_lower() {
//     icm20649.begin_I2C(CHASSIS_IMU_ADDR);
//     init();
    
// }

// ICM20649::ICM20649_upper() {
//     icm20649.begin_I2C(GIMBAL_IMU_ADDR);
//     init();
// }

void ICM20649::init(int bus) {
    // icm20649 = *(new Adafruit_ICM20649);
    icm20649.begin_I2C(bus);
    icm20649.setAccelRange(ICM20649_ACCEL_RANGE);
    icm20649.setGyroRange(ICM20649_GYRO_RANGE);
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
    // timer_set(9);
    icm_accel->getEvent(&accel);
    data[0] = accel.acceleration.x;
    data[1] = accel.acceleration.y;
    data[2] = accel.acceleration.z;
    // Serial.printf("accel read time: %i\n", timer_info_us(9));

    // icm20649.readAcceleration(data[0], data[1], data[2]);
    // const uint8_t numbytes = 6; // Read Accel, gyro, temp, and 9 bytes of mag

    // Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
    //   icm20649.i2c_dev(), NULL, ADDRBIT8_HIGH_TOREAD, ICM20X_B0_ACCEL_XOUT_H, numbytes);

    // uint8_t buffer[numbytes];
    // data_reg.read(buffer, numbytes);

    // data[0] = buffer[0] << 8 | buffer[1];
    // data[1] = buffer[2] << 8 | buffer[3];
    // data[2] = buffer[4] << 8 | buffer[5];

    // rawGyroX = buffer[6] << 8 | buffer[7];
    // rawGyroY = buffer[8] << 8 | buffer[9];
    // rawGyroZ = buffer[10] << 8 | buffer[11];
}

void ICM20649::read_gyro() {
    // icm20649.readGyroscope(data[0], data[1], data[2]);
    // timer_set(9);
    icm_gyro->getEvent(&gyro);
    data[3] = gyro.gyro.x;
    data[4] = gyro.gyro.y;
    data[5] = gyro.gyro.z;
    // Serial.printf("gyro read time: %i\n", timer_info_us(9));
}
