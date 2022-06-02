#ifndef REF_SYS_H
#define REF_SYS_H

#include "state/state.h"

class ref_sys{

    public:

    ref_sys();

    void init(S_RefSystem *tempInput);

    bool read_serial();

    S_RefSystem * run_data;

    private:



};

#endif
