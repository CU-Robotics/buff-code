#include "global_robot_state.h"
#include "motor_drivers/rm_can_interface.h"

class Gimbal {
    public:
        Gimbal(GlobalRobotState* globalRobotState, RM_CAN_Interface* rmCAN);
        bool ready() { return true; };
        void loop(int deltaTime);

        void aim(float yaw, float pitch);
        
    private:
        RM_CAN_Interface* rmCAN;

        float aimYaw;
        float aimPitch;

        float yawOffset;
        float pitchOffset;
};