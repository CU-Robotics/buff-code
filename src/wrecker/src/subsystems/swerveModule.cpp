#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"

#include "algorithms/PID_Filter.h"

SwerveModule::SwerveModule() {

}

void SwerveModule::setup(C_SwerveModule *data, S_Robot *r_state, S_SwerveModule *modState) {
  config = data;
  state = r_state;

  moduleState = modState;

  this->steerMotor.init(config->steerMotorID, 1, config->steerEncoderID);
  this->driveMotor.init(config->driveMotorID, 2);
}

void SwerveModule::calibrate() {
  this->steerMotor.updateMotor();
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
  calibrated = true;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  // Convert sensor and input units
  float inputAngle = angle + config->absolute_offset;
  if (inputAngle < 0)
    inputAngle += 360;

  float rawSteerAngle = this->steerMotor.getAngle();
  if (calibrated) {
    float steeringDifference = this->prevRawSteerAngle - rawSteerAngle;
    if (steeringDifference < -180)
      this->steerRollover--;
    else if (steeringDifference > 180)
      this->steerRollover++;
  }
  this->prevRawSteerAngle = rawSteerAngle;
  float steerAngle = convertSteerAngle(rawSteerAngle);
  this->prevSteerAngle = steerAngle;

  float rpm = steerMotor.getRpm() / 100.0; // 100.0 is based on the gear ratio of the motor and the steer belt

  // Inversion logic
  int inversion = 1;
  float error = inputAngle - steerAngle;
  float shadow = error - 360.0;
  if (fabs(shadow) < error)
    error = shadow;
  if (abs(error) > 90) {
    inversion = -1;
    inputAngle -= 180;
    if (inputAngle < 0)
      inputAngle += 360;
  }

  // Steer Velocity PID
  config->steerPos.continuous = true;
  tmp_steerPos.R = inputAngle;
  PID_Filter(&config->steerPos, &tmp_steerPos, steerAngle, deltaTime); 

  // Steer Position PID
  tmp_steerVel.R = -tmp_steerPos.Y;
  PID_Filter(&config->steerVel, &tmp_steerVel, rpm, deltaTime);

  if (tmp_steerVel.Y > 1.0)
    tmp_steerVel.Y = 1.0;
  if (tmp_steerVel.Y < -1.0)
    tmp_steerVel.Y = -1.0;

  // Drive Velocity PID
  moduleState->driveVel.R = speed * 4000;
  PID_Filter(&config->driveVel, &moduleState->driveVel, driveMotor.getRpm(), deltaTime);

  // Set motor power
  if (calibrated) {
    steerMotor.setPower(tmp_steerVel.Y);

    // Only drive if sufficiently close to target angle
    if (abs(inputAngle - steerAngle) > 20.0)
      driveMotor.setPower(moduleState->driveVel.Y * inversion);
    else
      driveMotor.setPower(0.0);
  }
}

int SwerveModule::findCalibrationMatch(int currValue, int* alignmentTable, int tableSize) {
  int smallestDistance = 360;
  int bestOffset = 0;
  for (int i = 0; i < tableSize; i++) {
    int distance = abs(currValue - alignmentTable[i]);
    if (distance < smallestDistance) {
      smallestDistance = distance;
      bestOffset = alignmentTable[i];
    }
  }
  return bestOffset;
}

int SwerveModule::getSteerId() {
  return config->steerMotorID;
}

float SwerveModule::convertSteerAngle(float rawSteerAngle) {
  float steerAngle = ((rawSteerAngle - this->steerOffset + (this->steerRollover * 360)) * (9.0/25.0));

  steerAngle = fmod(steerAngle, 360.0);
  if (steerAngle < 0) {
    steerAngle += 360;
  }
  return steerAngle;
}