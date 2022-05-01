#include "subsystem.h"
#include "../state/config.h"
#include "../state/state.h"

#include "../drivers/c620.h"
#include "../algorithms/PIDController.h"

class SwerveChassis: public Subsystem {
 public:
    void setup(C_SwerveModule config, S_SwerveChassis *state);
    void loop(float deltaTime);

  private:
    S_SwerveChassis *state;
    c610Enc steerMotor;
    c620CAN driveMotor;
};