#include <Arduino.h>

#include "swerveModule.h"

#include "../state/config.h"
#include "../state/state.h"

void SwerveModule::setup(C_SwerveModule *config, S_Robot *state) {
  this->config = config;
  this->state = state;
}

void SwerveModule::calibrate() {
  
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  // VEL PID
  float rpm = this->steerMotor.getRpm();

  // POS PID
  float rawPos = this->steerMotor.getAngle();
  float pos = ((rawPos + this->steerRollover * 360) - this->steerOffset) * (9.0/25.0);

  if ((rawPos - this->steerPrevAngle) > 180) {
    this->steerRollover--;
    pos = ((rawPos + this->steerRollover * 360) - this->steerOffset) * (9.0/25.0);
  } else if ((this->steerPrevAngle - rawPos) > 180) {
    this->steerRollover++;
    pos = ((rawPos + this->steerRollover * 360) - this->steerOffset) * (9.0/25.0);
  }

  int posTemp = int(pos * 100);
  posTemp = posTemp % 36000;
  pos = posTemp / 100.0;

  pos -= 45;

  if (pos < 0) {
    pos += 360;
  }

  float newPower = steerPosPID.calculate(pos, angle, deltaTime);
  float velPower = steerVelPID.calculate(rpm, -newPower * 10000, deltaTime);
  steerMotor.setPower(velPower);

  this->steerPrevAngle = rawPos;
}