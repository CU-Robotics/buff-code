#include "Subsystem.h"
#include "RobotInput.h"
#include "RobotConfig.h"
#include "M2006.h"
#include "Snail.h"
#include "PIDController.h"

class 17mmShooter: public Subsystem {
  public:
    InfantryShooter();
    void init(RobotConfig constants);
    void loop(RobotInput input, float deltaTime);

  private:
    M2006 feederMotor;
    Snail topFlywheelMotor;
    Snail bottomFlywheelMotor;

    PIDController feederController;
};