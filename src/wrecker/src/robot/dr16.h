#define DR16_H

#ifndef STRUCTS_H
#include "structs.h"
#endif

class dr16 {
    private:
        RobotInput * input;
        byte buf[14];
    public:
        dr16();
        void init(RobotInput *tempInput);
        void update();
};