#include "state/state.h"
#include "state/config.h"
#include "drivers/gm6020.h"
#include "drivers/MPU6050.h"
#include "algorithms/PID_Filter.h"

#ifndef GIMBAL_H
#define GIMBAL_H

class Gimbal 
{
 public:
    Gimbal();
    void setup(C_Gimbal *data, S_Robot *r_state);
    void update(float deltaTime);

  private:
    S_Robot* state;
    C_Gimbal* config;

    gm6020 yawMotor; 
    gm6020 pitchMotor;

    MPU6050 imu;

    float calibrated;
    float yawRollover;
    float prevRawYawAngle;

    float aimYaw = 0;
    float aimPitch = 0;

    int mouseReleased = 0;
    float gyroAngle = 0;

    float realizeYawEncoder(float rawAngle);
    float realizePitchEncoder(float rawAngle);

    //counters for updating imu
    unsigned long newTime = 0;
    unsigned long oldTime = 0;

};

#endif // GIMBAL_H