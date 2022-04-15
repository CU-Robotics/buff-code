#include "gimbal.h"

gimbal::gimbal(RobotConfig *tempConfig, RobotInput *tempInput) {
    config = tempConfig;
    input = tempInput;
}

gimbal::init() {
    pitchMotor.init(config.pitchID, );
    yawMotor.init(config.yawID, );
}

gimbal::update(float deltaTime) {

}