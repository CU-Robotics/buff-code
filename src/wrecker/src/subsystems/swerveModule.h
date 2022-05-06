#include "subsystem.h"
#include "../state/config.h"
#include "../state/state.h"

#include "../drivers/c620.h"
#include "../algorithms/PIDController.h"

class SwerveModule: public Subsystem {
 public:
    void setup(C_SwerveModule *config, S_Robot *state);
    void calibrate();
    void update(float speed, float angle, float deltaTime);

  private:
    C_SwerveModule *config;
    c610Enc steerMotor;
    c620CAN driveMotor;

    float steerAngle;
    float steerOffset;
    float steerRollover;

    void findCalibrationMatch();
    void motorAngleToWheelAngle();
};