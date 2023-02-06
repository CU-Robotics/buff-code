#include "global_robot_state.h"
#include "motor_drivers/rm_can_interface.h"

class Gimbal {
    public:
        Gimbal(GlobalRobotState* globalRobotState);
        bool ready();
        void loop(int deltaTime);

        void aim(float yaw, float pitch);
        
    private:
        RM_CAN_Device yawMotor1;
        RM_CAN_Device yawMotor2;
        RM_CAN_Device pitchMotor;

        float aimYaw;
        float aimPitch;

        float yawOffset;
        float pitchOffset;
};