#define RMMOTOR_H

#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "rmMotor.h"

short rmMotor::getTorque() {
    return torque;
}

short rmMotor::getRpm() {
    return rpm;
}

short rmMotor::getAngle() {
    return angle;
}

byte rmMotor::getTemp() {
    return temp;
}