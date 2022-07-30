#include "state/state.h"
#include "state/config.h"

#ifndef PID_FILTER_H
#define PID_FILTER_H

void PID_Filter(C_PID* config, S_PID* state, float feedback, long dt);

#endif // PID_FILTER_H