#ifndef GIMBAL_H
#define GIMBAL_H

#include "gm6020.h"

class gimbal {
    public:
        gimbal(RobotConfig *tempConfig, RobotInput *tempInput);
        void init();
        void update(float deltaTime);
    private:
        RobotConfig *config;
        RobotInput *input;

        gm6020 pitchMotor();
        gm6020 yawMotor();
}

#endif