#include "RobotInput.h"
#include "RobotConfig.h"

class Subsystem {
  public:
    void init(RobotConfig constants);
    void loop(RobotInput input, float deltaTime);

  private:
    RobotConfig *config;
};