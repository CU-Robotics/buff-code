#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"

#include "algorithms/PID_Filter.h"

SwerveModule::SwerveModule() {

}

void SwerveModule::setup(C_SwerveModule *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  switch (config->moduleID) {
    case 0:
      moduleState = &state->chassis.FL;
      break;
    case 1:
      moduleState = &state->chassis.FR;
      break;
    case 2:
      moduleState = &state->chassis.RR;
      break;
    case 3:
      moduleState = &state->chassis.RL;
      break;
    default:
      moduleState = &state->chassis.FL;
      break;
  }

  this->steerMotor.init(config->moduleID, 1, config->moduleID);
}

void SwerveModule::calibrate() {
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  //Serial.println("Entered swervemodule update");

  // POS PID CALCULATIONS
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
  
  // VEL PID CALCULATIONS
  //Serial.println("VEL PID CALCULATIONS");
  float rpm = steerMotor.getRpm() / 100.0;
  this->prevSteerAngle = steerAngle;

  // tmp_steerPos.R = angle;
  // tmp_steerPos.Y = steerAngle;
  // PID_Filter(&config->steerPos, &tmp_steerPos, deltaTime);
  config->steerVel.K[0] = 100;

  tmp_steerVel.R = speed * 150; //-tmp_steerPos.Y * 10000;
  tmp_steerVel.Y = rpm;
  PID_Filter(&config->steerVel, &tmp_steerVel, deltaTime);
  
  Serial.print(tmp_steerVel.R);
  Serial.print(" ... ");
  Serial.print(tmp_steerVel.Y);
  Serial.print(" ... ");
  Serial.println(rpm);

  steerMotor.setPower(tmp_steerVel.Y);
  steerMotor.updateMotor();
  
  //PID_Filter(&config->driveVel, &moduleState->driveVel, deltaTime);

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
