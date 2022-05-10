#include <Arduino.h>

#include "gimbal.h"

#include "../state/config.h"
#include "../state/state.h"

#include "../drivers/gm6020.h"

#include "../algorithms/PIDController.h"

void Gimbal::setup(C_SwerveChassis *config, S_Robot *state) {
  this->config = config;
  this->state = state;

  this->pitchController.init(this->config->pitchP, this->config->pitchF, this->config->pitchI, this->config->pitchF);
  this->yawController.init(this->config->yawP, this->config->yawI, this->config->yaw, this->config->pitchF);
}

void Gimbal::update(float deltaTime) {
  this->pitchSetpoint += this->state->s_driverInput.mouseY * this->sensitivity;
  this->yawSetpoint += this->state->s_driverInput.mouseX * this->sensitivity;

  if (this->pitchSetpoint > this->config->maxPitch) {
      this->pitchSetpoint = this->config->maxPitch;
  } else if (this->pitchSetpoint < this->config->minPitch) {
      this->pitchSetpoint = this->config->minPitch;
  }

  float pitchAngle = realizePitchEncoder(pitchMotor.getAngle());
  float yawAngle = realizeYawEncoder(yawMotor.getAngle())

  float pitchPower = this->pitchController.calculate(pitchAngle, pitchSetpoint, deltaTime);
  float yawPower = this->yawController.calculate(yawAngle, yawSetpoint, deltaTime);
}

float Gimbal::realizeYawEncoder(float angle) {
    return (angle - this->config->yawOffset) * 0.5;
}

float Gimbal::realizePitchEncoder(float angle) {
    return (angle - this->config->pitchOffset);
}