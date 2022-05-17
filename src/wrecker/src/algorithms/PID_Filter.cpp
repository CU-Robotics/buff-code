#include <Arduino.h>

#include "algorithms/PID_Filter.h"

void PID_Filter(C_PID* config, S_PID* state, float feedback, long dt)
{
  if (dt == 0.0)
    return;

  float error = state->R - feedback;
  
  // Derivative term = change in error (X[0])
  // state->X[2] = (error - state->X[0]) / dt;
  
  // Integral term = sum of error
  // state->X[1] = max(config->Imin, min(config->Imax, state->X[1] + (state->X[0] * dt)));
  
  // Proportional term = error (R - Y)
  state->X[0] = error;
  if (config->continuous) {
    float shadow = 360.0 - fabs(error);
    if (shadow < error) {
      if (state->R >= state->Y)
        state->X[0] = -shadow;
    }
  }

  // Sum terms, clamp, and return
  float signal = (config->K[0] * state->X[0]) + (config->K[1] * state->X[1]) + (config->K[2] * state->X[2]);
  state->Y = max(config->Ymin, min(config->Ymax, signal));
}





