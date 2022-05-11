#define C620_H

#include <FreqMeasureMulti.h>

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#ifndef RMMOTOR_H
#include "drivers/rmMotor.h"
#endif

// extern CAN_message_t c6x0Messages[3][2];

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