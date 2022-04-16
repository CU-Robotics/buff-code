#ifndef GM6020_H
#include "gimbal.h"

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

gimbal::gimbal(RobotConfig *tempConfig, RobotInput *tempInput) {
    config = tempConfig;
    input = tempInput;
}

gimbal::init() {
    pitchMotor.init(config.pitchID, superStructureSendMsg);
    yawMotor.init(config.yawID, superStructureSendMsg);
}

gimbal::update(float deltaTime) {

}

#endif