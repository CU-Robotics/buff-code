#include <Arduino.h>

#include "swerveChassis.h"

#include "swerveModule.h"
#include "../state/config.h"
#include "../state/state.h"

void SwerveChassis::setup(C_SwerveChassis *config, S_Robot *state) {
  this->config = config;
  this->state = state;

  float drivebaseRadius = sqrt(pow(this->config->drivebaseLength, 2) + pow(this->config->drivebaseWidth, 2));
  this->drivebaseConstant = this->config->drivebaseLength / drivebaseRadius;

  this->calibrate();
}

void SwerveChassis::update(float deltaTime) {
  float driveX = (this->state->s_driverInput.w - this->state->s_driverInput.a) * cos(this->state->s_gimbal.yaw);
  float driveY = (this->state->s_driverInput.w - this->state->s_driverInput.a) * sin(this->state->s_gimbal.yaw);
  drive(0, 0, 0.5, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {
  float gimbalAngle = this->state->s_gimbal.yaw;

  float newDriveX = -driveX * cos(radiansToDegrees(gimbalAngle)) + -driveY * sin(radiansToDegrees(-gimbalAngle));
  float newDriveY = -driveX * sin(radiansToDegrees(gimbalAngle)) + -driveY * cos(radiansToDegrees(gimbalAngle));

  float A = driveX - spin * this->drivebaseConstant;
  float B = driveX + spin * this->drivebaseConstant;
  float C = driveY - spin * this->drivebaseConstant;
  float D = driveY + spin * this->drivebaseConstant;

  float speedFR = sqrt(pow(B, 2) + pow(D, 2));
  float speedFL = sqrt(pow(B, 2) + pow(C, 2));
  float speedBL = sqrt(pow(A, 2) + pow(C, 2));
  float speedBR = sqrt(pow(A, 2) + pow(D, 2));
  float speedMax = std::max({speedFR, speedFL, speedBL, speedBR});

  if (speedMax > 1) {
    speedFR /= speedMax;
    speedFL /= speedMax;
    speedBL /= speedMax;
    speedBR /= speedMax;
  }

  float angleFR = radiansToDegrees(atan2(B, D));
  float angleFL = radiansToDegrees(atan2(B, C));
  float angleBL = radiansToDegrees(atan2(A, C));
  float angleBR = radiansToDegrees(atan2(A, D));

  this->moduleFR.update(speedFR, angleFR, deltaTime);
  this->moduleFL.update(speedFL, angleFL, deltaTime);
  this->moduleBL.update(speedBL, angleBL, deltaTime);
  this->moduleBR.update(speedBR, angleBR, deltaTime);
}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}