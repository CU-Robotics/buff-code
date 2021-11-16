#ifndef RMMOTOR_H
#include "rmMotor.h"
#endif

#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

class c620 : public rmMotor {
    private:
        short id;
        CAN_message_t& sendMsg;
        int byteNum;
    public:
        c620(short tempID, CAN_message_t* msg);
        void setPower(float power);
        void updateMotor();     //this will need to have information passed to it in some way, either by accepting each variable or by taking in a can message object
};
