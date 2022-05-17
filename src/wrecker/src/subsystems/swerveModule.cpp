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
}

void SwerveModule::calibrate() {
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  //Serial.println("Entered swervemodule update");

  float inputAngle = angle;
  if (inputAngle < 0) {
    inputAngle += 360;
  }

  // POS PID CALCULATIONS
  float rawSteerAngle = this->steerMotor.getAngle();

  float steeringDifference = this->prevRawSteerAngle - rawSteerAngle;

  //Serial.print("steeringDifference: ");
  //Serial.println(steeringDifference);

  if (steeringDifference < -180) {
    this->steerRollover--;
  } else if (steeringDifference > 180) {
    this->steerRollover++;
  }
  this->prevRawSteerAngle = rawSteerAngle;

  float steerAngle = ((rawSteerAngle + this->steerRollover * 360) - this->steerOffset) * (9.0/25.0);
  
  if (steerAngle < 0) {
    steerAngle += 360;
  }
  this->prevSteerAngle = steerAngle;
  
  // VEL PID CALCULATIONS
  //Serial.println("VEL PID CALCULATIONS");
  float rpm = steerMotor.getRpm() / 100.0;

  config->steerPos.continuous = true;
  tmp_steerPos.R = inputAngle;//speed * 360; //angle;
  PID_Filter(&config->steerPos, &tmp_steerPos, steerAngle, deltaTime); 

  tmp_steerVel.R = -tmp_steerPos.Y; //(speed * 300) - 150; //-tmp_steerPos.Y * 10000;
  PID_Filter(&config->steerVel, &tmp_steerVel, rpm, deltaTime);
  if (tmp_steerVel.Y > 1.0) {
    tmp_steerVel.Y = 1.0;
  }

  if (tmp_steerVel.Y < -1.0) {
    tmp_steerVel.Y = -1.0;
  }

  Serial.print(inputAngle);
  Serial.print(" ... ");
  Serial.print(speed);
  Serial.print(" ... ");
  // Serial.print(tmp_steerPos.R);
  // Serial.print(" ... ");
  Serial.print(rpm);
  Serial.print(" ... ");
  Serial.println(tmp_steerVel.Y);

  //Serial.println(config->steerPos.K[0]);
  //Serial.println(config->steerPos.continuous);

  // Serial.print(config->steerMotorID);
  // Serial.print(" ... ");
  // Serial.print(tmp_steerVel.Y);
  // Serial.print(" ... ");
  // Serial.println(rawSteerAngle);

  steerMotor.setPower(tmp_steerVel.Y);
  // steerMotor.setPower(0.5);
  steerMotor.updateMotor();
  
  //PID_Filter(&config->driveVel, &moduleState->driveVel, deltaTime);
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