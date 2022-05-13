#include "state/state.h"
#include "state/config.h"
#include "drivers/gm6020.h"
#include "algorithms/PID_Filter.h"

#ifndef GIMBAL_H
#define GIMBAL_H

class Gimbal 
{
 public:
    Gimbal();
    void setup(C_Gimbal *data, S_Robot *r_state);
    void update(float deltaTime);

    float realizeYawEncoder(float angle);
    float realizePitchEncoder(float angle);

  private:
    S_Robot* state;
    C_Gimbal* config;


    float yaw_Reference;
    float pitch_Reference;

    float pitchSum;
    float yawSum;


    gm6020 yawMotor; 
    gm6020 pitchMotor;
};

#endif // GIMBAL_H