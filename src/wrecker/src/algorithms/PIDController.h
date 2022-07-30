#define PIDCONTROLLER_H

class PIDController {
  public:
    // Configures the PID controller with tuned gain constants. Includes an optional "FeedForward" parameter.
    PIDController();
    // PIDController();

    void init(float kP, float kI, float kD);
    void init(float kP, float kI, float kD, float kF);

    // Returns the result of a PID calculation. Must pass in a current position, a setpoint, and a deltaTime (time since this controller's calculate() was last called).
    float calculate(float pos, float setpoint, float deltaTime);

    // Sets the range of values that calculate() can return. Default is (-1.0, 1.0).
    void setOutputRange(float low, float high);

    // Sets the range of maximum value that the integral term can output. Default is (-0.25, 0.25).
    void setIntegratorRange(float low, float high);

    // Enables continuous input, for use in rotational mechanisms. Input is the maximum encoder value. The minimum encoder value must be zero. This feature is off by default.
    void enableContinuousInput(float max);
    
  private:
    float kP = 0.0;
    float kI = 0.0;
    float kD = 0.0;
    float kF = 0.0;

    float integralSum = 0;
    float prevError = 0;

    float outputRangeLow = -1.0;
    float outputRangeHigh = 1.0;

    float integratorRangeLow = -0.25;
    float integratorRangeHigh = 0.25;

    bool continuousInput = false;
    float continuousInputMax = 360.0;
};