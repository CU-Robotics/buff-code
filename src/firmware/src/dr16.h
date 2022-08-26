#include "Arduino.h"

// #include "algorithms/Buffers.h"


#ifndef DR16_H
#define DR16_H

class DR16 {
    public:
        DR16();
        bool read(byte*);
        
    private:
        unsigned long d_t;
};

#endif // DR16_H