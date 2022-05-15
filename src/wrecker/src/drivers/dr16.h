#include "Arduino.h"

#include "state/state.h"


#ifndef DR16_H
#define DR16_H

class dr16 {
    public:
        dr16();
        void init(DriverInput *tempInput);
        void update();
    private:
        DriverInput* input;
        byte buf[18];
        unsigned long lastTime;
        int numBytes;
};

#endif // DR16_H