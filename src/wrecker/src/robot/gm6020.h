#ifndef RMMOTOR_H
#include "rmMotor.h"
#endif

#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

class gm6020 : public rmMotor {
    private:
    short id;
        CAN_message_t& sendMsg;
        int byteNum;
    public:
        gm6020(short tempID, CAN_message_t* msg);
        void setPower(float power);
}