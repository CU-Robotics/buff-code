#include "subsystem.h"

#ifndef CONFIG_H
#include "state/config.h"
#endif

#ifndef STATE_H
#include "state/state.h"
#endif

#include "drivers/c620.h"
#include "algorithms/PIDController.h"

class SwerveModule: public Subsystem {
 public:
    SwerveModule();
    void setup(C_SwerveModule *config, S_Robot *state);
    void update(float speed, float angle, float deltaTime);
    void calibrate();

  private:
    C_SwerveModule *config;
    S_Robot *state;
    c610Enc steerMotor;
    c620CAN driveMotor;

    float steerAngle;
    float steerOffset;
    float steerRollover;

    void findCalibrationMatch();
    void motorAngleToWheelAngle();
};