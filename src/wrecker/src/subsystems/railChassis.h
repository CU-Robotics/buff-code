#include "state/state.h"
#include "state/config.h"

#include "drivers/c620.h"

#ifndef RAIL_CHASSIS_H
#define RAIL_CHASSIS_H

class RailChassis {
 public:
    RailChassis();
    void setup(C_RailChassis *config, S_Robot *r_state);
    void update(unsigned long deltaTime);

  private:
    S_Robot *state;
    C_RailChassis *config;

    c620CAN leftDriveMotor;
    c620CAN rightDriveMotor;

    bool calibrated = false;

    float pos = 0.0;
    int currNode = 0;

    float leftOffset = 0;
    float leftRollover = 0;
    float leftPrev = 0;
    float rightRollover = 0;
    float rightOffset = 0;
    float rightPrev;

    float trackingTimeout = 0;

        float yaw_reference_prev = 0;
    float pitch_reference_prev = 0;


    float rampedSpeed = 0.0;

    void selectNode();
    float realizePosition(float leftRawPos, float rightRawPos);
};

#endif // RAIL_CHASSIS_H