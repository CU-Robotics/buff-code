// #ifndef GIMBAL_H
#define GIMBAL_H

#ifndef PIDCONTROLLER_H
#include "PIDController.h"
#endif

#ifndef STRUCTS_H
#include "structs.h"
#endif

#include "gm6020.h"

class gimbal {
    public:
        gimbal(RobotConfig *tempConfig, RobotInput *tempInput);
        void init(CAN_message_t* sendMsg);
        void update(float deltaTime);
    private:
        RobotConfig *config;
        RobotInput *input;

        gm6020 pitchMotor;
        gm6020 yawMotor;

        PIDController pitchController;
        PIDController yawController;
};

// #endif