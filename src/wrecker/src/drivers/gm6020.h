#include <FlexCAN_T4.h>

#include "rmMotor.h"


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

#endif // GM6020_H