#include "gimbal.h"

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

gimbal::gimbal(RobotConfig *tempConfig, RobotInput *tempInput) {
    config = tempConfig;
    input = tempInput;
}

void gimbal::init(CAN_message_t* sendMsg) {
    pitchController.init(config->pitchP, config->pitchI, config->pitchD);
    yawController.init(config->yawP, config->yawI, config->yawD);

    // this->pitchMotor = new gm6020();
    pitchMotor.init(config->pitchID, sendMsg);
    yawMotor.init(config->yawID, sendMsg);
}

void gimbal::update(float deltaTime) {
    float newPitchPower = pitchController.calculate(pitchMotor.getAngle(), input->gimbalPitch, deltaTime);
    float newYawPower = pitchController.calculate(yawMotor.getAngle(), input->gimbalPitch, deltaTime);

    pitchMotor.setPower(newPitchPower);
    yawMotor.setPower(newYawPower);
}

