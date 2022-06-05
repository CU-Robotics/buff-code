#include <Arduino.h>

#include "gimbal.h"
#include "state/state.h"
#include "state/config.h"
#include "drivers/gm6020.h"
#include "drivers/MPU6050.h"
#include "algorithms/PID_Filter.h"

Gimbal::Gimbal() {

}

void Gimbal::setup(C_Gimbal *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  this->yawMotor.init(6, 2);
  this->pitchMotor.init(7, 2);

  this->imu.init();

  config->yaw_PID.continuous = true;
  config->yaw_PID.K[0] = 1.65;
  config->yaw_PID.K[1] = 3.3;
  config->yaw_PID.K[2] = 0.0;//50.0;//0.55;
  config->pitch_PID.K[0] = 0.02;

  config->pitchMax = 50.0;
  config->pitchMin = -18.0;
  config->pitchOffset = 180;
  config->yawOffset = 330;
}

void Gimbal::update(float deltaTime) {
  newTime = millis();
  if ((newTime - oldTime) > 25) {
    imu.update_MPU6050();
    oldTime = newTime;
  }

  if (state->driverInput.b && !calibrated)
    calibrated = true;

  float rawYawAngle = yawMotor.getAngle();
  if (calibrated) {
    float yawDifference = this->prevRawYawAngle - rawYawAngle;
    if (yawDifference < -180)
      this->yawRollover--;
    else if (yawDifference > 180)
      this->yawRollover++;
  }
  this->prevRawYawAngle = rawYawAngle;

  // this->imu.update_MPU6050();
  this->gyroAngle += ((this->imu.get_gyro_x() - 0.7763) * (180.0 / M_PI) * 6) / deltaTime;

  float yawAngle = realizeYawEncoder(rawYawAngle);
  float pitchAngle = realizePitchEncoder(pitchMotor.getAngle());

  // Calculate new gimbal setpoints
  if (state->driverInput.mouseRight) {
    aimYaw += state->gimbal.yaw_reference;
    aimPitch += state->gimbal.pitch_reference;
    mouseReleased = 1;
  }
  else if (mouseReleased){
    aimYaw = state->gimbal.yaw_PID.Y;
    aimPitch = state->gimbal.pitch_PID.Y;
    mouseReleased = 0;
  }
  else {
    aimYaw += state->driverInput.mouseX * config->sensitivity * deltaTime / 10000.0; // 10000 is an arbitrary number -- do not change or sensitivies will change
    aimPitch += state->driverInput.mouseY * config->sensitivity * deltaTime / 10000.0; // 10000 is an arbitrary number -- do not change or sensitivies will change
  }
  

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

  this->state->gimbal.yawGlobal = yawAngle;

  // Yaw PID
  state->gimbal.yaw_PID.R = aimYaw;
  PID_Filter(&config->yaw_PID, &state->gimbal.yaw_PID, yawAngle, deltaTime);

  // Pitch PID
  state->gimbal.pitch_PID.R = aimPitch;
  PID_Filter(&config->pitch_PID, &state->gimbal.pitch_PID, pitchAngle, deltaTime);

  // Set motor power
  if (calibrated) {
    float move = state->driverInput.mouseX * config->sensitivity * deltaTime / 10000.0;
    Serial.println(move);
    yawMotor.setPower(move);
    //yawMotor.setPower(state->gimbal.yaw_PID.Y);
    //pitchMotor.setPower(state->gimbal.pitch_PID.Y);
  }
}

float Gimbal::realizeYawEncoder(float rawAngle) {
  float yawAngle = ((rawAngle - config->yawOffset + (this->yawRollover * 360)) * 0.5) + (this->gyroAngle);

  yawAngle = fmod(yawAngle, 360.0);
  if (yawAngle < 0)
    yawAngle += 360;
  return yawAngle;
}

float Gimbal::realizePitchEncoder(float rawAngle) {
    return (rawAngle - config->pitchOffset);
}