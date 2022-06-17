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

  pitchFilter.init(35);
  yawFilter.init(50);
}

void Gimbal::update(float deltaTime) {
  if (state->driverInput.b && !calibrated) {
    calibrated = true;
    imu.update_MPU6050();
    this->state->gimbal.gyroDrift = this->imu.get_gyro_z();
  }

  // Gyro management
  newTime = millis();
  if ((newTime - oldTime) > 25 && calibrated) {
    imu.update_MPU6050();
    oldTime = newTime;
  }
  float gyroSpeed = (this->imu.get_gyro_z() - this->state->gimbal.gyroDrift) * (180.0 / M_PI) * (deltaTime / 1000000.0); // There are 1000000 microseconds in a second.
  this->state->gimbal.gyroAngle += gyroSpeed * this->state->chassis.spin;

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
  this->state->gimbal.yawGlobal = realizeYawEncoderWithoutGyro(rawYawAngle); // Set the global yaw
  
  // Pitch encoder
  float pitchAngle = realizePitchEncoder(pitchMotor.getAngle());

  // Calculate gimbal setpoints
  if (state->driverInput.mouseRight) {
    if (state->gimbal.yaw_reference != yaw_reference_prev || state->gimbal.pitch_reference != pitch_reference_prev) {
      aimYaw = yawAngle;
      aimPitch = pitchAngle;
      aimYaw += state->gimbal.yaw_reference;
      aimPitch += state->gimbal.pitch_reference;
      yaw_reference_prev = state->gimbal.yaw_reference;
      pitch_reference_prev = state->gimbal.pitch_reference;
    }
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

    float movePitch = state->driverInput.mouseY * config->sensitivity * deltaTime;
    mouseYFilter.push(movePitch);
    aimPitch -= mouseYFilter.mean();
  }
  
  // Pich softstops
  if (aimPitch < config->pitchMin)
    aimPitch = config->pitchMin;
  else if (aimPitch > config->pitchMax)
    aimPitch = config->pitchMax;


  // Yaw PID
  state->gimbal.yawPos.R = aimYaw;
  PID_Filter(&config->yawPos, &state->gimbal.yawPos, yawAngle, deltaTime);

  yawFilter.push(yawMotor.getRpm() * 0.5);
  state->gimbal.yawVel.R = state->gimbal.yawPos.Y;
  PID_Filter(&config->yawVel, &state->gimbal.yawVel, yawFilter.mean(), deltaTime);

  float dynamicYawFeedforward = -gyroSpeed * 1.0;

  // Pitch PID
  state->gimbal.pitchPos.R = aimPitch;
  PID_Filter(&config->pitchPos, &state->gimbal.pitchPos, pitchAngle, deltaTime);

  pitchFilter.push(pitchMotor.getRpm());
  state->gimbal.pitchVel.R = state->gimbal.pitchPos.Y;
  PID_Filter(&config->pitchVel, &state->gimbal.pitchVel, pitchFilter.mean(), deltaTime);

  float dynamicPitchFeedForward = cos((PI / 180.0) * pitchAngle) * 0.45;

  // Set motor power
  if (calibrated) {
    // yawMotor.setPower(state->gimbal.yawVel.Y + dynamicYawFeedforward);
    // pitchMotor.setPower(state->gimbal.pitchVel.Y + dynamicPitchFeedForward);
    yawMotor.setPower(0.0);
    pitchMotor.setPower(0.0);

    Serial.print(yawMotor.getAngle());
  }
}

float Gimbal::realizeYawEncoder(float rawAngle) {
  float yawAngle = ((rawAngle - config->yawOffset + (this->yawRollover * 360)) * 0.5) + (this->state->gimbal.gyroAngle);
  return yawAngle;
}

float Gimbal::realizeYawEncoderWithoutGyro(float rawAngle) {
  float yawAngle = ((rawAngle - config->yawOffset + (this->yawRollover * 360)) * 0.5);
  return yawAngle;
}

float Gimbal::realizePitchEncoder(float rawAngle) {
    return (rawAngle - config->pitchOffset);
}