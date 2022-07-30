#include <iostream>
#include <cmath>
#include "pid.h"

PID::PID(double dt, double min, double max, double Kp, double Ki, double Kd) : 
    _dt(dt),
    _min(min),
    _max(max),
    _Kp(Kp),
    _Ki(Ki),
    _Kd(Kd),
    _initError(0),
    _integral(0)
{}

double PID::calculate(double setPoint, double pv) {
    double error = setPoint - pv;

    double Pout = _Kp*error;
    
    _integral += error * _dt;
    double Iout = _Ki*_integral;

    double derivative = (error - _initError) / _dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    if (output > _max) {output = _max;}
    else if (output < _min) {output = _min;}

    _initError = error;
    return output;
}
