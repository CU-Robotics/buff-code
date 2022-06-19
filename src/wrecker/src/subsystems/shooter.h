#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "drivers/flywheel.h"

#ifndef SHOOTER_H
#define SHOOTER_H

class Shooter {
 public:
    Shooter();
    void setup(C_Shooter17 *config, S_Robot *state);
    void update(unsigned long deltaTime);

  private:
    S_Robot *state;
    C_Shooter17 *config;

    c620CAN feedMotor;
    flywheel topFlywheel;
    flywheel bottomFlywheel;

    float calibrated = false;

    int shooterTimer = 0;
    bool shooterClear = false;
    bool armed = false;

    void fire(float speed);
};

#endif // SHOOTER_H