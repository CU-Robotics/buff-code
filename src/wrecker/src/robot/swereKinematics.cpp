#include "swerveKinematics.h"
#include <math.h>
#include "constants.h"

#define _USE_MATH_DEFINES

swerveKinematics::swerveKinematics(){
    flWSpd, WheelSpd[0] = 0;
    frWSpd, WheelSpd[1] = 0;
    rlWSpd, WheelSpd[2] = 0;
    rrWSpd, WheelSpd[3] = 0;

    flWAgl, WheelAgl[0] = 0;
    frWAgl, WheelAgl[1] = 0;
    rlWAgl, WheelAgl[2] = 0;
    rrWAgl, WheelAgl[3] = 0;
}

swerveKinematics::swerveKinematics(int flSpd, int frSpd, int rlSpd, int rrSpd, int flAgl, int frAgl, int rlAgl, int rrAgl){
    flWSpd, WheelSpd[0] = flSpd;
    frWSpd, WheelSpd[1] = frSpd;
    rlWSpd, WheelSpd[2] = rlSpd;
    rrWSpd, WheelSpd[3] = rrSpd;

    flWAgl, WheelAgl[0] = flAgl;
    frWAgl, WheelAgl[1] = frAgl;
    rlWAgl, WheelAgl[2] = rlAgl;
    rrWAgl, WheelAgl[3] = rrAgl;
}

double swerveKinematics::flWheelSpeed(){
    return flWSpd;
}
double swerveKinematics::frWheelSpeed(){
    return frWSpd;
}
double swerveKinematics::rlWheelSpeed(){
    return rlWSpd;
}
double swerveKinematics::rrWheelSpeed(){
    return rrWSpd;
}

double swerveKinematics::flWheelAngle(){
    return flWAgl;
}
double swerveKinematics::frWheelAngle(){
    return frWAgl;
}
double swerveKinematics::rlWheelAngle(){
    return rlWAgl;
}
double swerveKinematics::rrWheelAngle(){
    return rrWAgl;
}

void swerveKinematics::calculate(double x, double y, double angle){
    double A = x - angle * (WHEEL_BASE_LENGTH / SWERVE_R); //define constant
    double B = x + angle * (WHEEL_BASE_LENGTH / SWERVE_R);
    double C = y - angle * (WHEEL_BASE_LENGTH / SWERVE_R);
    double D = y + angle * (WHEEL_BASE_LENGTH / SWERVE_R);


    //WHeel Speed
    flWSpd = sqrt((B * B) + (C * C));
    frWSpd = sqrt((B * B) + (D * D));
    rlWSpd = sqrt((A * A) + (C * C));
    rrWSpd = sqrt((A * A) + (D * D));

    

    //Wheel Angle
    flWAgl = atan2(B, C)*180/M_PI;
    frWAgl = atan2(B, D)*180/M_PI;
    rlWAgl = atan2(A, D)*180/M_PI;
    rrWAgl = atan2(A, C)*180/M_PI;
    

    WheelSpd[0] = flWSpd;
    WheelSpd[1] = frWSpd;
    WheelSpd[2] = rlWSpd;
    WheelSpd[3] = rrWSpd;

    WheelAgl[0] = flWAgl;
    WheelAgl[1] = frWAgl;
    WheelAgl[2] = rlWAgl;
    WheelAgl[3] = rrWAgl;
}

void swerveKinematics::optimize(double flPID, double frPID, double rlPID, double rrPID){
    //Optimizes motor rotation for obtuse angles
    if (flWAgl >= 180) {
        flWAgl -= 180;
        flWSpd *= -1;
    }
    
    if (frWAgl >= 180) {
        frWAgl -= 180;
        frWSpd *= -1;
    }

    if (rlWAgl >= 180) {
        rrWAgl -= 180;
        rrWSpd *= -1;
    }

    if (rrWAgl >= 180) {
        rrWAgl -= 180;
        rrWSpd *= -1;
    }
}
