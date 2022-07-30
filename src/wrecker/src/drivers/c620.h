#include <FlexCAN_T4.h>
#include <FreqMeasureMulti.h>

#include "rmMotor.h"


#ifndef C620_H
#define C620_H

void sendC6x0();

class c620CAN : public rmMotor {
    public:
        c620CAN();
        void init(uint8_t motorId, uint8_t tempCanBusNum);
        void setPower(float power);
        void updateMotor();
};

class c610Enc : public rmMotor {
    private:
        uint8_t inPin;
        FreqMeasureMulti freq;
        float angle;
    public:
        c610Enc();
        void init(short tempID, uint8_t tempCanBusNum, uint8_t encPin);
        void setPower(float power);
        float getAngle();
        void updateMotor();
};

#endif