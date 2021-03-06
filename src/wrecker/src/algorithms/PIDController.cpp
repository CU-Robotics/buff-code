#include "PIDController.h"
// #include <cmath>  //this isn't in arduino (maybe)
#include <math.h>

PIDController::PIDController() {

};

// PIDController::PIDController(float kP, float kI, float kD) {
//   this->kP = kP;
//   this->kI = kI;
//   this->kD = kD;
// }

// PIDController::PIDController(float kP, float kI, float kD, float kF) {
//   this->kP = kP;
//   this->kI = kI;
//   this->kD = kD;
//   this->kF = kF;
// }

void PIDController::init(float kP, float kI, float kD) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
}

void PIDController::init(float kP, float kI, float kD, float kF) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->kF = kF;
}


float PIDController::calculate(float pos, float setpoint, float deltaTime) {
  float error = setpoint - pos;
  if (this->continuousInput) {
    float oppositeError =  error - this->continuousInputMax;
    if (fabs(oppositeError) < error) {
      error = oppositeError;
    }
  }

  // Proportional term
  float pTerm = kP * error;

  // Integral term
  this->integralSum += error * deltaTime;
  float iTerm = kI * this->integralSum;
  if (iTerm > this->integratorRangeHigh) {
    iTerm = this->integratorRangeHigh;
  } else if (iTerm < this->integratorRangeLow) {
    iTerm = this->integratorRangeLow;
  }

  /*if (error < 0.5) {
    this->integralSum = 0;
  }*/

  // Derivative term
  float dTerm = kD * ((error - this->prevError) / deltaTime);
  this->prevError = error;

  // Feedforward term
  float fTerm = kF;

  // Sum terms, clamp, and return
  float output = pTerm + iTerm + dTerm + fTerm;
  if (output > this->outputRangeHigh) {
    output = this->outputRangeHigh;
  } else if (output < this->outputRangeLow) {
    output = this->outputRangeLow;
  }
  return output;
}

void PIDController::setOutputRange(float low, float high) {
  this->outputRangeLow = low;
  this->outputRangeHigh = high;
}

void PIDController::setIntegratorRange(float low, float high) {
  this->integratorRangeLow = low;
  this->integratorRangeHigh = high;
}

void PIDController::enableContinuousInput(float max) {
  this->continuousInput = true;
  this->continuousInputMax = max;
}