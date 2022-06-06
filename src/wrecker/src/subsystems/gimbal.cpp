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
  mouseXFilter.init(50);
  mouseYFilter.init(50);

  config->yawVel.Ymin = -150.0;
  config->yawVel.Ymax = 150.0;
  config->yawVel.K[0] = 0.01;

  config->yawPos.continuous = true;
  config->yawPos.K[0] = 1.5;

  config->pitchMax = 50.0;
  config->pitchMin = -18.0;
  config->pitchOffset = 180;
  config->yawOffset = 330;
}

void Gimbal::update(float deltaTime) {
  if (state->driverInput.b && !calibrated) {
    calibrated = true;
    imu.update_MPU6050();
    gyroDrift = this->imu.get_gyro_x();
  }

  // Gyro management
  newTime = millis();
  if ((newTime - oldTime) > 25 && calibrated) {
    imu.update_MPU6050();
    oldTime = newTime;
  }
  this->gyroAngle += ((this->imu.get_gyro_x() - gyroDrift) * (180.0 / M_PI) * 6) / deltaTime;

  // Yaw encoder
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
  this->state->gimbal.yawGlobal = yawAngle; // Set the global yaw
  
  // Pitch encoder
  float pitchAngle = realizePitchEncoder(pitchMotor.getAngle());

  // Calculate gimbal setpoints
  if (state->driverInput.mouseRight) {
    aimYaw -= state->gimbal.yaw_reference;
    aimPitch += state->gimbal.pitch_reference;
    mouseReleased = 1;
  }
  else if (mouseReleased){
    aimYaw = yawAngle;
    aimPitch = pitchAngle;
    mouseReleased = 0;
  }
  else {
    float moveYaw = state->driverInput.mouseX * config->sensitivity * deltaTime;
    mouseXFilter.push(moveYaw);
    aimYaw += mouseXFilter.mean();
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


  // Yaw PID
  state->gimbal.yawPos.R = aimYaw;
  PID_Filter(&config->yawPos, &state->gimbal.yawPos, yawAngle, deltaTime);

  state->gimbal.yawVel.R = state->gimbal.yawPos.Y;
  PID_Filter(&config->yawVel, &state->gimbal.yawVel, (yawMotor.getRpm() * 0.5), deltaTime);

  Serial.print(aimYaw);
  Serial.print(" - ");
  Serial.print(yawAngle);
  Serial.print(" - ");
  Serial.print(state->gimbal.yawPos.Y);
  Serial.print(" - ");
  Serial.print(state->gimbal.yawVel.Y);
  Serial.println();


  // Pitch PID
  // state->gimbal.pitch_PID.R = aimPitch;
  // PID_Filter(&config->pitch_PID, &state->gimbal.pitch_PID, pitchAngle, deltaTime);

  // Set motor power
  if (calibrated) {
    yawMotor.setPower(state->gimbal.yawVel.Y);
    //yawMotor.setPower(-0.5);
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