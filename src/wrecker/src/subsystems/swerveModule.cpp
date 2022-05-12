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
  Serial.print("Setup:\n"); dump_Swerve(moduleState, "ModuelState");
  steerMotor.init(config->moduleID, 1, config->moduleID);
}

void SwerveModule::calibrate() {
  steerOffset = findCalibrationMatch(steerMotor.getAngle(), config->alignment, 9);
  steerRollover = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  //Serial.println("Entered swervemodule update");

  // POS PID CALCULATIONS
  float rawSteerAngle = steerMotor.getAngle();

  if ((rawSteerAngle - prevRawSteerAngle) > 180) {
    steerRollover--;
  } else if ((prevRawSteerAngle - rawSteerAngle) > 180) {
    steerRollover++;
  }

  float steerAngle = ((rawSteerAngle + steerRollover * 360) - steerOffset) * (9.0/25.0);

  int tempSteerAngle = int(steerAngle * 100);
  tempSteerAngle = tempSteerAngle % 36000;
  steerAngle = tempSteerAngle / 100.0;
  
  if (steerAngle < 0) {
    steerAngle += 360;
  }
  
  // VEL PID CALCULATIONS
  float rpm = (steerAngle - prevSteerAngle) * deltaTime * 60000000;
  prevSteerAngle = steerAngle;

  moduleState->steerPos.Y = 0;
  moduleState->steerVel.Y = rpm;

  moduleState->steerVel.R = -moduleState->steerPos.Y * 10000;

  PID_Filter(&config->steerPos, &moduleState->steerPos, deltaTime);

  PID_Filter(&config->steerVel, &moduleState->steerVel, deltaTime);
  
  PID_Filter(&config->driveVel, &moduleState->driveVel, deltaTime);

  prevRawSteerAngle = rawSteerAngle;
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
