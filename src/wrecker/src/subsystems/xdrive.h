#define X_DRIVE_H

#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "algorithms/PID_Filter.h"


#ifndef X_DRIVE 
#define X_DRIVE

class XDrive {
 public:
    XDrive(C_Robot *r_config, S_Robot *r_state);
    void update(float deltaTime);

  private:
    C_Robot *config;
    S_Robot *state;
    S_PID driveVel;
    c620CAN driveMotors[4];
};

#endif // SWERVE_MODULE