#include "Arduino.h"
#include "drivers/MPU6050.h"

MPU6050 imu;

unsigned long time1 = 0;
unsigned long time2 = 0;

void setup() {
    imu.init();
    Serial.begin(115200);
}

void loop() {
    time1 = micros();
    imu.update_MPU6050();
    time2 = micros();
    Serial.println(imu.get_gyro_x());
    Serial.println(time2-time1);
    Serial.println("------------");
    delay(1000);
}