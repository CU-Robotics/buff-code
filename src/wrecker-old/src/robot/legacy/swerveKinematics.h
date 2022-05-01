#include "chassis.h"

double wheelBaseLength;
double wheelBaseWidth;
double SWERVE_R;

class swerveKinematics : chassis {
    public:
        swerveKinematics();
        swerveKinematics(int flSpd, int frSpd, int rlSpd, int rrSpd, int flAgl, int frAgl, int rlAgl, int rrAgl);

        void calculate(double x, double y, double angle);
        double flWheelSpeed();
        double frWheelSpeed();
        double rlWheelSpeed();
        double rrWheelSpeed();

        double flWheelAngle();
        double frWheelAngle();
        double rlWheelAngle();
        double rrWheelAngle();

        void optimize(double flPID, double frPID, double rlPID, double rrPID);

    private:
        double flWSpd;
        double frWSpd;
        double rlWSpd;
        double rrWSpd;

        double flWAgl;
        double frWAgl;
        double rlWAgl;
        double rrWAgl;

        double WheelSpd[4];
        double WheelAgl[4];
};