#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "drivers/flywheel.h"

#ifndef SHOOTER42_H
#define SHOOTER42_H

extern flywheel fw_2;

class Shooter42 {
 public:
    Shooter42();
    void setup(C_Shooter42 *config, S_Robot *state);
    void update(unsigned long deltaTime);

  private:
    S_Robot *state;
    C_Shooter42 *config;

    c620CAN feedMotor;

    float calibrated = false;
    int pos;
    int posRollover;
    int prevPos;
    int feedAngle;
    bool mouseUp;

    int shooterTimer = 0;
    bool shooterClear = false;
    bool armed = false;

    void fire();
};

#endif // SHOOTER42_H