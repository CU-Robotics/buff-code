#include <Arduino.h>

#include "swerveChassis.h"

#include "swerveModule.h"
#include "../state/config.h"
#include "../state/state.h"

void SwerveChassis::setup(C_SwerveChassis *config, S_Robot *state) {
  this->config = config;
  this->state = state;

  this->drivebaseRadius = sqrt(pow(this->config->drivebaseLength, 2) + pow(this->config->drivebaseWidth, 2));

  this->calibrate();
}

void SwerveChassis::loop(float deltaTime) {
  drive(0, 0, 0.5, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {
  float A = driveX - spin * (this->config->drivebaseLength / this->drivebaseRadius);
  float B = driveX + spin * (this->config->drivebaseLength / this->drivebaseRadius);
  float C = driveY - spin * (this->config->drivebaseWidth / this->drivebaseRadius);
  float D = driveY + spin * (this->config->drivebaseWidth / this->drivebaseRadius);

  float speedFR = sqrt(pow(B, 2) + pow(C, 2));
  float speedFL = sqrt(pow(B, 2) + pow(D, 2));
  float speedBL = sqrt(pow(A, 2) + pow(D, 2));
  float speedBR = sqrt(pow(A, 2) + pow(C, 2));
  float speedMax = std::max({speedFR, speedFL, speedBL, speedBR});

  if (speedMax > 1) {
    speedFR /= speedMax;
    speedFL /= speedMax;
    speedBL /= speedMax;
    speedBR /= speedMax;
  }

  float angleFR = radiansToDegrees(atan2(B, C));
  float angleFL = radiansToDegrees(atan2(B, D));
  float angleBL = radiansToDegrees(atan2(A, D));
  float angleBR = radiansToDegrees(atan2(A, C));

  this->moduleFR.update(speedFR, angleFR, deltaTime);
  this->moduleFL.update(speedFL, angleFL, deltaTime);
  this->moduleBL.update(speedBL, angleBL, deltaTime);
  this->moduleBR.update(speedBR, angleBR, deltaTime);
}

void SwerveChassis::driveSimple(float driveX, float driveY, float deltaTime) {

}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}