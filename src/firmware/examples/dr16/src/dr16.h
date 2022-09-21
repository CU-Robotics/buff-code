#include "Arduino.h"

#ifndef DR16_H
#define DR16_H

class DR16 {
    public:
        DR16();
        void read(byte*, int);
        
    private:
        unsigned long d_t;
};

#endif // DR16_H