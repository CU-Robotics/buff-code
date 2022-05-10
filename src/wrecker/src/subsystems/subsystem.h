#ifndef CONFIG_H
#include "state/config.h"
#endif

#ifndef STATE_H
#include "state/state.h"
#endif

class Subsystem {
  public:
    Subsystem();
    void setup(C_Config *config, S_Robot *state);
    void update(float deltaTime);

  private:
    C_Config *config;
    S_Robot *state;
};