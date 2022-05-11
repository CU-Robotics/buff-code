#ifndef RMMOTOR_H
#include "rmMotor.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#ifndef GM6020_H
#define GM6020_H

class gm6020 : public rmMotor {
    public:
        gm6020();
        void init(short tempID, uint8_t tempCanBusNum);
        void setPower(float power);
    private:
        short id;
        int byteNum;
};

#endif