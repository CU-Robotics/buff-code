#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "railChassis.h"
#include "drivers/c620.h"
#include "algorithms/PID_Filter.h"

RailChassis::RailChassis() {
  
}

void RailChassis::setup(C_RailChassis *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  leftDriveMotor.init(1, 1);
  rightDriveMotor.init(2, 1);

  // PID Tuning
  data->drivePos.Ymax = 1;
  data->drivePos.Ymin = -1;
  data->driveVel.Imin = -20000;
  data->driveVel.Imax = 20000;

  data->drivePos.K[0] = 0.0001; // 0.00015
  data->drivePos.K[1] = 0.0;
  data->drivePos.K[2] = 0.0;

  data->driveVel.K[0] = 0.0006;
  data->driveVel.K[1] = 0.0;
  data->driveVel.K[2] = 0.0;
  //data->drivePos.K[2] = 400000;
}

void RailChassis::update(unsigned long deltaTime) {

  if (state->driverInput.s2 == 2 && !calibrated) {
    calibrated = true;
    leftOffset = leftDriveMotor.getAngle();
    rightOffset = rightDriveMotor.getAngle();
  } else {
    calibrated = false;
  }

  float leftRawPos = leftDriveMotor.getAngle();
  float rightRawPos = rightDriveMotor.getAngle();
  float leftDifference = this->leftPrev - leftRawPos;
  if (leftDifference < -180)
    this->leftRollover--;
  else if (leftDifference > 180)
    this->leftRollover++;

  float rightDifference = this->rightPrev - rightRawPos;
  if (rightDifference < -180)
    this->rightRollover--;
  else if (rightDifference > 180)
    this->rightRollover++;
  this->leftPrev = leftRawPos;
  this->rightPrev = rightRawPos;

  pos = realizePosition(leftDriveMotor.getAngle(), rightDriveMotor.getAngle());
  // Serial.print("measured "); Serial.println(pos);

  if(abs(pos - this->config->nodes[this->currNode]) < config->acceptanceRange)
    selectNode();

  // PID
  state->railChassis.drivePos.R = this->config->nodes[this->currNode];
  // Serial.print("Reference "); Serial.println(state->railChassis.drivePos.R);
  PID_Filter(&config->drivePos, &state->railChassis.drivePos, pos, deltaTime);

  float speed = state->railChassis.drivePos.Y;
  if (rampedSpeed < speed) {
    rampedSpeed += (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed > speed) {
      rampedSpeed = speed;
    }
  } else if (rampedSpeed > speed) {
    rampedSpeed -= (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed < speed) {
      rampedSpeed = speed;
    }
  }

  state->railChassis.driveVel.R = rampedSpeed * 4000;
  PID_Filter(&config->driveVel, &state->railChassis.driveVel, leftDriveMotor.getRpm(), deltaTime);

  Serial.print(pos);
  Serial.print(" - ");
  Serial.print(state->railChassis.drivePos.Y);
  Serial.print(" - ");
  Serial.print(speed);
  Serial.print(" - ");
  Serial.print(rampedSpeed);
  Serial.print(" - ");
  Serial.print(state->railChassis.driveVel.Y);
  Serial.println();

  // Set motor output
  if (state->driverInput.s2 == 2 && !state->gimbal.tracking){
    // Set motor output
    if (calibrated) {
      leftDriveMotor.setPower(state->railChassis.driveVel.Y);
      rightDriveMotor.setPower(state->railChassis.driveVel.Y);
    }
  }
  else{
    leftDriveMotor.setPower(0);
    rightDriveMotor.setPower(0);
  }
}


void RailChassis::selectNode() {
  int newNode = random(0, this->config->numNodes);
  this->currNode = newNode;
}

float RailChassis::realizePosition(float leftRawPos, float rightRawPos) {
  float leftPos = leftRawPos - leftOffset + (this->leftRollover * 360);
  float rightPos = rightRawPos - rightOffset + (this->rightRollover * 360);
  return ((leftPos + rightPos) / 2.0);
}