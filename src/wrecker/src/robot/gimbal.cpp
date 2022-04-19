#include "gimbal.h"

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

gimbal::gimbal(RobotConfig *tempConfig, RobotInput *tempInput) {
    config = tempConfig;
    input = tempInput;
}

void gimbal::init(CAN_message_t* sendMsg) {
    // this->pitchMotor = new gm6020();
    pitchMotor.init(config->pitchID, sendMsg);
    yawMotor.init(config->yawID, sendMsg);
}

void gimbal::update(float deltaTime) {

}

