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
        int id;
        CAN_message_t& sendMsg;
        int byteNum;
    public:
        c620(int tempID, CAN_message_t* msg);
        void setPower(float power);
};
