#include <FlexCAN_T4.h>
#include <FreqMeasureMulti.h>

#include "rmMotor.h"


#ifndef C620_H
#define C620_H

class c620CAN : public rmMotor {
    public:
        c620CAN();
        void init(uint8_t motorId, uint8_t tempCanBusNum);
        void setPower(float power);
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
};

#endif // C620_H