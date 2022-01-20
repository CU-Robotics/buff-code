#ifndef RMMOTOR_H
#include "rmMotor.h"
#endif

#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

class c620CAN : public rmMotor {
    private:
        int id;
        CAN_message_t& sendMsg;
        int byteNum;
    public:
        c620CAN(int tempID, CAN_message_t* msg);
        void setPower(float power);
};


class c620PWM : public rmMotor {
    private:
        uint8_t outPin;
        uint8_t inPin;
    public:
        c620PWM(uint8_t input, uint8_t output);
        void setPower(float power);
}