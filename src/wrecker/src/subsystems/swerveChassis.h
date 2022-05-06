#include "subsystem.h"
#include "swerveModule.h"
#include "../state/config.h"
#include "../state/state.h"

class SwerveChassis: public Subsystem {
 public:
    void setup(C_SwerveChassis *config, S_Robot *state);
    void loop(float deltaTime);

  private:
    C_SwerveChassis *config;
    S_Robot *state;

    SwerveModule moduleFR;
    SwerveModule moduleFL;
    SwerveModule moduleBL;
    SwerveModule moduleBR;

    float drivebaseRadius;

    void calibrate();
    void drive(float driveX, float driveY, float spin, float deltaTime);
    void driveSimple(float driveX, float driveY, float deltaTime);
    float radiansToDegrees(float radians);
};