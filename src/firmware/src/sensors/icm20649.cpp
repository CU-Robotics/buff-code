#include "icm20649.h"

icm20649::icm20649() {
    // start i2c communications
    icm.begin_I2C();
    
    // get the sensor objects linked ot the sensor
    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAcellerometerSensor();
    icm_gyro = icm.getGyroSensor();
    
    
}
