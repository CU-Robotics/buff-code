#include <Arduino.h>

#include "gimbal.h"
#include "state/state.h"
#include "state/config.h"
#include "drivers/gm6020.h"
#include "algorithms/PID_Filter.h"

Gimbal::Gimbal() {
  
}

void Gimbal::setup(C_Gimbal *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  this->yawMotor.init(config->yawMotorID, 2);
  this->pitchMotor.init(config->pitchMotorID, 2);
}

void Gimbal::update(float deltaTime) {
  if (state->driverInput.b && !calibrated) {
    calibrated = true;
  }

  float rawYawAngle = yawMotor.getAngle();
  if (calibrated) {
    float yawDifference = this->prevRawYawAngle - rawYawAngle;
    if (yawDifference < -180)
      this->yawRollover--;
    else if (yawDifference > 180)
      this->yawRollover++;
  }
  this->prevRawYawAngle = rawYawAngle;

  float yawAngle = realizeYawEncoder(rawYawAngle);
  float pitchAngle = realizeYawEncoder(pitchMotor.getAngle());


  // Calculate new gimbal setpoints
  aimYaw += state->driverInput.mouseX * config->sensitivity * deltaTime / 10000.0; // 10000 is an arbitrary number -- do not change or sensitivies will change
  aimPitch += state->driverInput.mouseY * config->sensitivity * deltaTime / 10000.0; // 10000 is an arbitrary number -- do not change or sensitivies will change

  // Yaw angle range correction
  aimYaw = fmod(aimYaw, 360.0);
  if (aimYaw < 0)
    aimYaw += 360;

  // Pitch angle range correction
  aimPitch = fmod(aimPitch, 360.0);
  if (aimPitch < 0)
    aimPitch += 360;
  // Pich softstops
  if (aimPitch < config->pitchMin)
    aimPitch = config->pitchMin;
  else if (aimPitch > config->pitchMax)
    aimPitch = config->pitchMax;

  // Yaw PID
  state->gimbal.yaw_PID.R = aimYaw;
  PID_Filter(&config->yaw_PID, &state->gimbal.yaw_PID, yawAngle, deltaTime);

  // Pitch PID
  state->gimbal.pitch_PID.R = aimPitch;
  PID_Filter(&config->pitch_PID, &state->gimbal.pitch_PID, pitchAngle, deltaTime);

  // Set motor power here?
  yawMotor.setPower(state->gimbal.yaw_PID.Y);
  pitchMotor.setPower(state->gimbal.pitch_PID.Y);
}

float Gimbal::realizeYawEncoder(float rawAngle) {
  float yawAngle = ((rawAngle - config->yawOffset + (this->yawRollover * 360)) * 0.5);

  yawAngle = fmod(yawAngle, 360.0);
  if (yawAngle < 0)
    yawAngle += 360;
  return yawAngle;
}

float Gimbal::realizePitchEncoder(float rawAngle) {
    return (rawAngle - config->pitchOffset);
}