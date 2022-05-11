#include "subsystem.h"

#include "../state/config.h"
#include "../state/state.h"

#include "../drivers/gm6020.h"

#include "../algorithms/PIDController.h"

class Gimbal: public Subsystem {
 public:
    void setup(C_Gimbal *config, S_Robot *state);
    void loop(float deltaTime);

  private:
    C_Gimbal *config;
    S_Robot *state;

    float pitchSum;
    float yawSum;

    gm6020 pitchMotor;
    gm6020 yawMotor; 

    PIDController pitchController;
    PIDController yawController;

    float realizePitchEncoder(float angle);
    float realizeYawEncoder(float angle);
};