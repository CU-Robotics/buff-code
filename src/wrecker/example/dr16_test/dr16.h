#define DR16_H

#ifndef STRUCTS_H
#include "structs.h"
#endif

class dr16 {
    private:
        RobotInput * input;
        byte buf[18];
        unsigned long lastTime;
        int numBytes;
    public:
        dr16();
        void init(RobotInput *tempInput);
        void update();
};