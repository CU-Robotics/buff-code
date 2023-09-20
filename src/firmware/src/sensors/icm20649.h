// #include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
// #include <Adafruit_Sensor.h>

#ifndef BUFF_ICM20649_H
#define BUFF_ICM20649_H

#define ICM20649_DOF 6

#define CHASSIS_IMU_ADDR 0x68
#define GIMBAL_IMU_ADDR 0x69

// typedef enum {
//   ICM20649_ACCEL_RANGE_4_G,
//   ICM20649_ACCEL_RANGE_8_G,
//   ICM20649_ACCEL_RANGE_16_G,
//   ICM20649_ACCEL_RANGE_30_G,
// } icm20649_accel_range_t;
#define ICM20649_ACCEL_RANGE ICM20649_ACCEL_RANGE_16_G

// /** The gyro data range */
// typedef enum {
//   ICM20649_GYRO_RANGE_500_DPS,
//   ICM20649_GYRO_RANGE_1000_DPS,
//   ICM20649_GYRO_RANGE_2000_DPS,
//   ICM20649_GYRO_RANGE_4000_DPS,
// } icm20649_gyro_range_t;
#define ICM20649_GYRO_RANGE ICM20649_GYRO_RANGE_4000_DPS

class ICM20649 {
public:
    ICM20649();
    // ICM20649_lower();
    // ICM20649_upper();
    void init(int bus);
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