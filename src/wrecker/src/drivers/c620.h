#include <FreqMeasureMulti.h>

#ifndef RMMOTOR_H
#include "drivers/rmMotor.h"
#endif

#ifndef C620_H
#define C620_H

class c620CAN : public rmMotor {
    public:
        c620CAN();
        // c620CAN(short tempID, CAN_message_t* msg);
        void init(short motorId, CAN_message_t* msg);
        void setPower(float power);
};



class c610Enc : public rmMotor {
    private:
        uint8_t inPin;
        FreqMeasureMulti freq;
        float angle;
    public:
        c610Enc();
        // c610Enc(short tempID, CAN_message_t* msg, uint8_t encPin); //constructor
        void init(short tempID, CAN_message_t* msg, uint8_t encPin);
        void setPower(float power);
        float getAngle();
};

#endif // C620_H
