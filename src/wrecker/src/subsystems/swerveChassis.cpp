#include <Arduino.h>

#include "state/state.h"

#ifndef SWERVEMODULE_H
#include "swerveModule.h"
#endif

#include "state/config.h"
#include "swerveChassis.h"

SwerveChassis::SwerveChassis() {

}

void SwerveChassis::setup(C_SwerveChassis *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  drivebaseRadius = sqrt(pow(config->drivebaseLength, 2) + pow(config->drivebaseWidth, 2));

  calibrate();
}

void SwerveChassis::update(float deltaTime) {
  drive(0, 0, 0.5, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {
  float A = driveX - spin * (config->drivebaseLength / drivebaseRadius);
  float B = driveX + spin * (config->drivebaseLength / drivebaseRadius);
  float C = driveY - spin * (config->drivebaseWidth / drivebaseRadius);
  float D = driveY + spin * (config->drivebaseWidth / drivebaseRadius);

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

  moduleFR.update(speedFR, angleFR, deltaTime);
  moduleFL.update(speedFL, angleFL, deltaTime);
  moduleBL.update(speedBL, angleBL, deltaTime);
  moduleBR.update(speedBR, angleBR, deltaTime);
}

void SwerveChassis::driveSimple(float driveX, float driveY, float deltaTime) {

}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}

