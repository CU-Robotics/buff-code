#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"
#include "swerveChassis.h"

SwerveChassis::SwerveChassis() {

}

void SwerveChassis::setup(C_SwerveChassis *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  float drivebaseRadius = sqrt(pow(config->drivebaseLength, 2) + pow(this->config->drivebaseWidth, 2));
  drivebaseConstant = config->drivebaseLength / drivebaseRadius;

  calibrate();
}

void SwerveChassis::update(float deltaTime) {
  float driveX = (state->driverInput.w - state->driverInput.a) * cos(state->gimbal.yaw);
  float driveY = (state->driverInput.w - state->driverInput.a) * sin(state->gimbal.yaw);
  drive(0, 0, 0.5, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {

  float gimbalAngle = state->gimbal.yaw;

  float newDriveX = -driveX * cos(radiansToDegrees(gimbalAngle)) + -driveY * sin(radiansToDegrees(-gimbalAngle));
  float newDriveY = -driveX * sin(radiansToDegrees(gimbalAngle)) + -driveY * cos(radiansToDegrees(gimbalAngle));

  float A = driveX - spin * drivebaseConstant;
  float B = driveX + spin * drivebaseConstant;
  float C = driveY - spin * drivebaseConstant;
  float D = driveY + spin * drivebaseConstant;

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

  moduleFR.update(speedFR, angleFR, deltaTime);
  moduleFL.update(speedFL, angleFL, deltaTime);
  moduleBL.update(speedBL, angleBL, deltaTime);
  moduleBR.update(speedBR, angleBR, deltaTime);
}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}

