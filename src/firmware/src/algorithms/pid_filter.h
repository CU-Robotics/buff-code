#ifndef PID_FILTER_H
#define PID_FILTER_H

struct PIDFilter {
    float K[4] = {0.0, 0.0, 0.0, 0.0}; // P, I, D, F
    float sumError = 0.0;
    float prevError = 0.0;
    float variableFeedforward = 1.0;

    float setpoint;
    float measurement;
    float output;

    float filter(int deltaTime) {
        float error = setpoint - measurement;
        sumError += error * deltaTime;
        output = (K[0] * error)
            + (K[1] * sumError)
            + (K[2] * ((error - prevError) / deltaTime))
            + (K[3] * variableFeedforward);
        prevError = error;
        return output;
    }

    void applyVariableFeedForward(float var) {
        float variableFeedforward = var;
    }
};

#endif