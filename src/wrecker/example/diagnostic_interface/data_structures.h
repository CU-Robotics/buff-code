#define DATA_STRUCTURES_H

struct PID
{
  public:
    float K[3] = {0.5f, 0.5f, 0.25f};
    float X[3] = {0.0f, 0.0f, 0.0f};
    float Y = 0.0f;
    float Ymax = 180.0f;
    float Ymin = -180.0f;
    float imax = 45.0f;
    float imin = -45.0f;
    bool continuous = false;
};

struct Servo
{
  public:
    int duty_max;
    int duty_min;
    int pin;
    
    float theta_max;
    float R;
    
    PID* pid;
};

struct BlinkingLED
{
  int pw_cycle = 50;
  int pw_period = 100;
  int pwm_ctr = 0;
  int pin = 13;
};
