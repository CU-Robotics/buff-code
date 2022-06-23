#ifndef FLYWHEEL_H
#define FLYWHEEL_H
#include <Arduino.h>
#include <Servo.h>

class flywheel {
    private:
        Servo esc;
        byte pinNum;
    public:
        flywheel();
        void init(byte tempPinNum);
        void setPower(float newPower);
        void reset();
};

#endif