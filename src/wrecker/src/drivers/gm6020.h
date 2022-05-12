#ifndef RMMOTOR_H
#include "rmMotor.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "state/state.h"

#ifndef GM6020_H
#define GM6020_H

class gm6020 : public rmMotor {
    public:
        gm6020();
        void init(short tempID, uint8_t tempCanBusNum);
        void setPower(float power);
        void updateMotor();
    private:
        short id;
        int byteNum;
};

#endif