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
  this->driveMotor.init(config->driveMotorID, 1);
}

void SwerveModule::calibrate() {
  this->steerMotor.updateMotor();
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
  calibrated = true;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  //Serial.println("Entered swervemodule update");

  float inputAngle = angle + config->absolute_offset;
  if (inputAngle < 0) {
    inputAngle += 360;
  }

  // POS PID CALCULATIONS
  float rawSteerAngle = this->steerMotor.getAngle();

  if (calibrated) {
    float steeringDifference = this->prevRawSteerAngle - rawSteerAngle;
    if (steeringDifference < -180) {
      this->steerRollover--;
    } else if (steeringDifference > 180) {
      this->steerRollover++;
    }
  }
  this->prevRawSteerAngle = rawSteerAngle;

  float steerAngle = convertSteerAngle(rawSteerAngle);
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

  // Serial.print(inputAngle);
  // Serial.print(" ... ");
  // Serial.print(speed);
  // Serial.print(" ... ");
  // Serial.print(tmp_steerPos.R);
  // Serial.print(" ... ");
  // Serial.print(rpm);
  // Serial.print(" ... ");
  // Serial.println(tmp_steerVel.Y);

  int calibMatch = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  // Serial.print(this->steerMotor.getAngle());
  // Serial.print(" ... ");
  // Serial.print(this->steerOffset);
  // Serial.print(" ... ");
  // Serial.print(this->steerRollover);
  // Serial.print(" ... ");
  // Serial.print(steerAngle);
  // Serial.print(" ... ");
  // Serial.println(calibMatch);

  //Serial.println(config->steerPos.K[0]);
  //Serial.println(config->steerPos.continuous);

  // Serial.print(config->steerMotorID);
  // Serial.print(" ... ");
  // Serial.print(tmp_steerVel.Y);
  // Serial.print(" ... ");
  // Serial.println(rawSteerAngle);

  if (calibrated) {
    //steerMotor.setPower(tmp_steerVel.Y);
    steerMotor.updateMotor();
    driveMotor.setPower(speed * 0.3);
    driveMotor.updateMotor();
    Serial.println(driveMotor.getRpm());
  }

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

float SwerveModule::convertSteerAngle(float rawSteerAngle) {
  float steerAngle = ((rawSteerAngle - this->steerOffset + (this->steerRollover * 360)) * (9.0/25.0));

  steerAngle = fmod(steerAngle, 360.0);
  if (steerAngle < 0) {
    steerAngle += 360;
  }
  return steerAngle;
}