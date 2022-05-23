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
    void setup(C_SwerveModule *config, S_Robot *state, S_SwerveModule *modState);
    void update(float speed, float angle, float deltaTime);
    void calibrate();
    int getSteerId();

  private:
    S_Robot *state;
    C_SwerveModule *config;
    S_SwerveModule *moduleState;

    S_PID tmp_steerVel;
    S_PID tmp_steerPos;

    c610Enc steerMotor;
    c620CAN driveMotor;

    float steerOffset = 0;
    float steerRollover = 0;
    float prevRawSteerAngle = 0;
    float prevSteerAngle = 0;

    bool calibrated = false;
    float absolute_offset = 0;

    int findCalibrationMatch(int currValue, int* alignmentTable, int tableSize);
    void motorAngleToWheelAngle();
    float convertSteerAngle(float rawSteerAngle);
};

#endif // SWERVE_MODULE