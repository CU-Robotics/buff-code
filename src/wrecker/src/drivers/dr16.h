#define DR16_H

#ifndef CONFIG_H
#include "state/config.h"
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