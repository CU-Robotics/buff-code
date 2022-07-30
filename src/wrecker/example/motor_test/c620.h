#ifndef C620_H
#define C620_H

#include <FreqMeasureMulti.h>

#include "rmMotor.h"


class c620CAN : public rmMotor {
    public:
        c620CAN(short tempID, CAN_message_t* msg);
        void setPower(float power);
};


class c620PWM : public rmMotor {
    private:
        uint8_t outPin;
        uint8_t inPin;
        FreqMeasureMulti freq;
        float angle;
    public:
        c620PWM(uint8_t input, uint8_t output); //constructor
        void setPower(float power);     //takes in a float from -1 to 1 and converts that to a PWM output
        float getAngle();   //retrieves angle from encoder and returns it
};

#endif // C620_H
