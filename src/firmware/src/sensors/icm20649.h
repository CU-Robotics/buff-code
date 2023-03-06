#include <Adafruit_ICM20649>

#ifndef BUFF_ICM20649_H
#define BUFF_ICM20649_H

class ICM20649 {
public:
    ICM20649();
private:
    Adafruit_ICM20649 icm20649;
    Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;
};

#endif
