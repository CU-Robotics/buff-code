#include "RobotInput.h"
#include "RobotConstants.h"

class Subsystem {
  public:
    void init();
    void loop(RobotInput input, RobotConstants constants);
};