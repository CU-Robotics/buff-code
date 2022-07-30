#include "state/state.h"

#ifndef REF_SYS_H
#define REF_SYS_H

class Ref_System{

    public:

    Ref_System();

    void init(S_RefSystem *tempInput);

    bool read_serial();

    private:

    S_RefSystem* state;
};

#endif
