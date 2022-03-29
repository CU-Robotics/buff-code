#include "Subsystem.h"
#include "RobotInput.h"
#include "M2006.h"
#include "Snail.h"

class InfantryShooter: public Subsystem {
  public:
    InfantryShooter();
    void init();
    void loop(RobotInput input);

  private:
    M2006 feederMotor;
    Snail topFlywheelMotor;
    Snail bottomFlywheelMotor;
};