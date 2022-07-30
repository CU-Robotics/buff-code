 #ifndef DATA_STRUCTURES_H
 #include "data_structures.h"
 #endif

void PID_Filter(float R, PID* pid, long dt)
{
  float error = R - pid->Y;
  
  // Derivative term
  pid->X[2] = (error - pid->X[0]) / dt;
  float dTerm = pid->K[2] * pid->X[2];
  
  // Integral term
  pid->X[1] = max(pid->imin, min(pid->imax, pid->X[1] + (pid->X[0] * dt)));
  float iTerm = pid->K[1] * pid->X[1];
  
  pid->X[0] = error;
  if (pid->continuous) {
    float shadow = 360.0 - fabs(error);
    if (shadow < pid->X[0]) {
      if (R >= pid->Y)
        pid->X[0] = -shadow;
    }
  }

  // Proportional term
  float pTerm = pid->K[0] * pid->X[0];
  // Feedforward term
  //float fTerm = pid->K[0] * pid->X[0];

  // Sum terms, clamp, and return
  pid->Y = max(pid->Ymin, min(pid->Ymax, pTerm + iTerm + dTerm));
}
