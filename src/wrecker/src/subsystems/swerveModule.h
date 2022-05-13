#define SWERVEMODULE_H

#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "algorithms/PID_Filter.h"


#ifndef SWERVE_MODULE 
#define SWERVE_MODULE

class SwerveModule {
 public:
    SwerveModule();
    void setup(C_SwerveModule* config, S_Robot* state);
    void update(float speed, float angle, float deltaTime);
    void calibrate();

  private:
    S_Robot* state;
    C_SwerveModule* config;
    S_SwerveModule* moduleState;

    c610Enc steerMotor;
    c620CAN driveMotor;

    float steerOffset = 0;
    float steerRollover = 0;
    float prevRawSteerAngle = 0;
    float prevSteerAngle = 0;

    int findCalibrationMatch(int currValue, int* alignmentTable, int tableSize);
    void motorAngleToWheelAngle();
};

#endif // SWERVE_MODULE