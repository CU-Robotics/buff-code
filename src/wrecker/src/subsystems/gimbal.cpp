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

  config->yawPos.K[0] = 2.3;
  config->yawPos.K[2] = 0.01;

  config->yawVel.Ymin = -150.0;
  config->yawVel.Ymax = 150.0;
  config->yawVel.K[0] = 0.06;

  config->pitchPos.K[0] = 3;
  config->pitchPos.K[2] = 0.0;

  config->pitchVel.K[0] = 0.012;

  config->pitchMax = 50.0;
  config->pitchMin = -18.0;
  config->pitchOffset = 177;
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
  float gyroSpeed = (this->imu.get_gyro_x() - gyroDrift) * (180.0 / M_PI) * (deltaTime / 1000000.0);
  this->gyroAngle += gyroSpeed;

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
    aimYaw += state->gimbal.yaw_reference;
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

    float movePitch = state->driverInput.mouseY * config->sensitivity * deltaTime;
    mouseYFilter.push(movePitch);
    aimPitch -= mouseYFilter.mean();
  }
  

  // // Yaw angle range correction
  // aimYaw = fmod(aimYaw, 360.0);
  // if (aimYaw < 0)
  //   aimYaw += 360;
  
  // Pich softstops
  if (aimPitch < config->pitchMin)
    aimPitch = config->pitchMin;
  else if (aimPitch > config->pitchMax)
    aimPitch = config->pitchMax;


  // Yaw PID
  state->gimbal.yawPos.R = aimYaw;
  PID_Filter(&config->yawPos, &state->gimbal.yawPos, yawAngle, deltaTime);

  yawFilter.push(yawMotor.getRpm() * 0.5);
  // int time = millis();
  // float modulate = (time % 1000) / 1000.0;
  // modulate = (25 * sinf(2 * PI * modulate)) + 100;
  state->gimbal.yawVel.R = state->gimbal.yawPos.Y;
  PID_Filter(&config->yawVel, &state->gimbal.yawVel, yawFilter.mean(), deltaTime);

  float dynamicYawFeedforward = -gyroSpeed * 1.0;


  // Serial.print(yawAngle);
  // Serial.print(" - ");
  // Serial.print(state->gimbal.yawPos.Y);
  // Serial.print(" - ");
  // Serial.print(state->gimbal.yawVel.R);
  // Serial.print(" - ");
  // Serial.print(yawFilter.mean());
  // Serial.print(" - ");
  // Serial.print(state->gimbal.yawVel.Y);
  // Serial.println();


  // Pitch PID
  state->gimbal.pitchPos.R = aimPitch;
  PID_Filter(&config->pitchPos, &state->gimbal.pitchPos, pitchAngle, deltaTime);

  pitchFilter.push(pitchMotor.getRpm());
  state->gimbal.pitchVel.R = state->gimbal.pitchPos.Y;
  PID_Filter(&config->pitchVel, &state->gimbal.pitchVel, pitchFilter.mean(), deltaTime);

  float dynamicPitchFeedForward = cos((PI / 180.0) * pitchAngle) * 0.45;

  // Serial.print(pitchAngle);
  // Serial.print(" - ");
  // Serial.print(aimPitch);
  // Serial.println();

  // Set motor power
  if (calibrated) {
    yawMotor.setPower(state->gimbal.yawVel.Y + dynamicYawFeedforward);
    //yawMotor.setPower(-0.5);
    //pitchMotor.setPower(state->gimbal.pitch_PID.Y);
    pitchMotor.setPower(state->gimbal.pitchVel.Y + dynamicPitchFeedForward);
  }
}

float Gimbal::realizeYawEncoder(float rawAngle) {
  float yawAngle = ((rawAngle - config->yawOffset + (this->yawRollover * 360)) * 0.5) + (this->gyroAngle);

  // yawAngle = fmod(yawAngle, 360.0);
  // if (yawAngle < 0)
  //   yawAngle += 360;
  return yawAngle;
}

float Gimbal::realizePitchEncoder(float rawAngle) {
    return (rawAngle - config->pitchOffset);
}