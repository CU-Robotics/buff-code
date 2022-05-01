#include "subsystem.h"
#include "../state/config.h"
#include "../state/state.h"

class SwerveChassis: public Subsystem {
 public:
    void setup(C_SwerveChassis config, S_SwerveChassis *state);
    void loop(float deltaTime);

  private:
    S_SwerveChassis *state;
};