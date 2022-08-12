#include "Arduino.h"

#include "algorithms/Buffers.h"


#ifndef DR16_H
#define DR16_H

class DR16 {
    public:
        int id = -1;

        DR16();
        void init(int);
        void read(HIDBuffer*);
    private:
        unsigned long d_t;
};

#endif // DR16_H