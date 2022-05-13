#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"
#include "algorithms/PID_Filter.h"
#include "drivers/serial_interface.h"


SwerveModule::SwerveModule() {

}

void SwerveModule::setup(C_SwerveModule* data, S_Robot* r_state, S_SwerveModule* sm_state) {
  config = data;
  state = r_state;

  moduleState = sm_state;
  steerMotor.init(config->steerMotorID, 1, config->steerEncoderID);
}

void SwerveModule::calibrate() {
  steerOffset = findCalibrationMatch(steerMotor.getAngle(), config->alignment, 9);
  steerRollover = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {

  // POS PID CALCULATIONS
  float rawSteerAngle = steerMotor.getAngle();

  float steeringDifference = prevRawSteerAngle - rawSteerAngle;

  if (steeringDifference < -180) {
    steerRollover--;
  } else if (steeringDifference > 180) {
    steerRollover++;
  }
  prevRawSteerAngle = rawSteerAngle;

  float steerAngle = ((rawSteerAngle + steerRollover * 360) - steerOffset) * (9.0/25.0);
  
  if (steerAngle < 0) {
    steerAngle += 360;
  }
  prevSteerAngle = steerAngle;
  
  // VEL PID CALCULATIONS
  //Serial.println("VEL PID CALCULATIONS");
  float rpm = steerMotor.getRpm() / 100.0;

  moduleState->steerPos.R = speed * 360; //angle;
  PID_Filter(&config->steerPos, &moduleState->steerPos, steerAngle, deltaTime); 

  moduleState->steerVel.R = -moduleState->steerPos.Y; //(speed * 300) - 150; //-tmp_steerPos.Y * 10000;
  PID_Filter(&config->steerVel, &moduleState->steerVel, rpm, deltaTime);
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