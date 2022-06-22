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
  data->drivePos.Ymax = 4000;
  data->drivePos.Ymin = -4000;
  data->driveVel.K[0] = 0.0006;
  data->drivePos.K[0] = 1;
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

  if(abs(pos - this->config->nodes[this->currNode]) < config->acceptanceRange)
    selectNode();

  // PID
  state->railChassis.drivePos.R = this->config->nodes[this->currNode];
  PID_Filter(&config->drivePos, &state->railChassis.drivePos, pos, deltaTime);

  if (state->gimbal.yaw_reference != yaw_reference_prev || state->gimbal.pitch_reference != pitch_reference_prev) {
    trackingTimeout = 0;
    Serial.println("bad 1");
  } else {
    trackingTimeout += deltaTime;
    Serial.println("good 1");
  }

  state->railChassis.driveVel.R = 0;

  if (trackingTimeout > 2000000) { // 1 sec
    state->railChassis.driveVel.R = state->railChassis.drivePos.Y;
  }

  PID_Filter(&config->driveVel, &state->railChassis.driveVel, leftDriveMotor.getRpm(), deltaTime);

  // Set motor output

  float speed = state->railChassis.driveVel.Y;
  if (rampedSpeed < speed) {
    rampedSpeed += (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed > speed) {
      rampedSpeed = speed;
    }
  } else if (rampedSpeed > speed) {
    rampedSpeed -=  (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed < speed) {
      rampedSpeed = speed;
    }
  }

  Serial.print(rampedSpeed);
  Serial.print(" - ");
  Serial.println(speed);

  // Set motor output
  if (calibrated) {
    leftDriveMotor.setPower(rampedSpeed);
    rightDriveMotor.setPower(rampedSpeed);
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