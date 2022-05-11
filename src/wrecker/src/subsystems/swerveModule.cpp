#include <Arduino.h>

#include "swerveModule.h"

#include "../state/config.h"
#include "../state/state.h"

void SwerveModule::setup(C_SwerveModule *config, S_Robot *state) {
  this->config = config;
  this->state = state;
}

void SwerveModule::calibrate() {
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  // VEL PID
  float rpm = this->steerMotor.getRpm();

  // POS PID
  float rawSteerAngle = this->steerMotor.getAngle();

  if ((rawSteerAngle - this->prevRawSteerAngle) > 180) {
    this->steerRollover--;
  } else if ((this->prevRawSteerAngle - rawSteerAngle) > 180) {
    this->steerRollover++;
  }

  float steerAngle = ((rawSteerAngle + this->steerRollover * 360) - this->steerOffset) * (9.0/25.0);

  int tempSteerAngle = int(steerAngle * 100);
  tempSteerAngle = tempSteerAngle % 36000;
  steerAngle = tempSteerAngle / 100.0;
  
  if (steerAngle < 0) {
    steerAngle += 360;
  }

  float newPower = steerPosPID.calculate(steerAngle, angle, deltaTime);
  float velPower = steerVelPID.calculate(rpm, -newPower * 10000, deltaTime);
  steerMotor.setPower(velPower);

  this->prevRawSteerAngle = rawSteerAngle;
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