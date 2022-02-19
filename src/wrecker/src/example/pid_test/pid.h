#ifndef _PID_H_
#define _PID_H_

class PID {
    private:
        double _dt; //Change in time interval
        double _min; // Min output
        double _max; // Max output
        double _Kp; // Proportional gain
        double _Ki; // Integral gain
        double _Kd; // Derivative gain
        double _initError; 
        double _integral;

    public:
        PID(double dt, double min, double max, double Kp, double Ki, double Kd);
        double calculate(double setpoint, double pv);
        //~PID();

};

#endif
