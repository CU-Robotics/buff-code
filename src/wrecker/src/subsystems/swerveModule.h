#include "subsystem.h"

#include "../state/config.h"
#include "../state/state.h"

#include "../drivers/c620.h"
#include "../algorithms/PIDController.h"

class SwerveModule: public Subsystem {
 public:
    void setup(C_SwerveModule *config, S_Robot *state);
    void update(float speed, float angle, float deltaTime);
    void calibrate();

  private:
    C_SwerveModule *config;
    S_Robot *state;

    c610Enc steerMotor;
    c620CAN driveMotor;

    PIDController steerVelPID;
    PIDController steerPosPID;
    PIDController drivePID;

    float steerPrevAngle;
    float steerOffset;
    float steerRollover;

    void findCalibrationMatch();
    void motorAngleToWheelAngle();
};