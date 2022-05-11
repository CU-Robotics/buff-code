#include "state/state.h"
#include "drivers/c620.h"
#include "state/config.h"
#include "algorithms/PID_Filter.h"


#ifndef SWERVE_MODULE 
#define SWERVE_MODULE

class SwerveModule {
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

#endif // SWERVE_MODULE