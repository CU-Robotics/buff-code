#include "state/state.h"
#include "state/config.h"

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

    void drive(float speed);
};

#endif // RAIL_CHASSIS_H