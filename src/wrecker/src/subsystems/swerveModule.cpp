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
      moduleState = &state->chassis.FR;
      break;
    case 1:
      moduleState = &state->chassis.FL;
      break;
    case 2:
      moduleState = &state->chassis.RL; 
      break;
    case 3:
      moduleState = &state->chassis.RR;
      break;
    }
}

void SwerveModule::calibrate() {
  
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  // VEL PID
  float rpm = steerMotor.getRpm();

  // POS PID
  float rawPos = steerMotor.getAngle();
  float pos = ((rawPos + steerRollover * 360) - steerOffset) * (9.0/25.0);

  if ((rawPos - steerPrevAngle) > 180) {
    steerRollover--;
    pos = ((rawPos + steerRollover * 360) - steerOffset) * (9.0/25.0);
  } else if ((steerPrevAngle - rawPos) > 180) {
    steerRollover++;
    pos = ((rawPos + steerRollover * 360) - steerOffset) * (9.0/25.0);
  }

  int posTemp = int(pos * 100);
  posTemp = posTemp % 36000;
  pos = posTemp / 100.0;

  pos -= 45;

  if (pos < 0) {
    pos += 360;
  }

  moduleState->steerPos.Y = pos;
  PID_Filter(&config->steerPos, &moduleState->steerPos, deltaTime);

  moduleState->steerVel.Y = rpm;
  //moduleState->steerVel.R = -moduleState->steerPos.Y * 10000;
  //PID_Filter(&config->steerVel, &moduleState->steerVel, deltaTime);

  //PID_Filter(&config->driveVel, &moduleState->driveVel, deltaTime);

  steerPrevAngle = rawPos;
}






