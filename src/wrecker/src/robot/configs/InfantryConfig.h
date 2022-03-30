#include "RobotConfig.h"

struct InfantryConfig : RobotConfig {
  public:
    struct 17mmShooter {
      public:
        float feedrate = 100; // rpm
        float flywheelPower = 0.5; // normalized
        struct PID {
          public:
            float P = 0.0;
            float I = 0.0;
            float D = 0.0;
            float F = 0.0;
        };
    };
};