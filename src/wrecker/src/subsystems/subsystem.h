#include "state/state.h"
#include "state/config.h"


#ifndef SUBSYSTEM
#define SUBSYSTEM

class Subsystem {
  public:
    Subsystem();
    //void setup(C_Config *config, S_Robot *state);
    void update(float deltaTime);

  private:
    //C_Config *config;
    //S_Robot *state;
};

#endif // SUBSYSTEM