#ifndef FLYWHEEL_H
#define FLYWHEEL_H
#include <Arduino.h>

class flywheel {
    private:
        byte pinNum;
    public:
        flywheel();
        void init(byte tempPinNum);
        void setPower(float newPower);
        void initCal(byte tempPinNum);
};

#endif