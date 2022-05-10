#define DR16_H

#ifndef CONFIG_H
#include "state/config.h"
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