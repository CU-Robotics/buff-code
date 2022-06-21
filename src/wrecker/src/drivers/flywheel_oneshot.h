#ifndef FLYWHEEL_ONESHOT_H
#define FLYWHEEL_ONESHOT_H
#include <Arduino.h>
#include <Servo.h>

class flywheel_oneshot {
    private:
        Servo esc;
        byte pinNum;
    public:
        flywheel_oneshot();
        void init(byte tempPinNum);
        void setPower(float newPower);
        // void initCal(byte tempPinNum);
};

#endif