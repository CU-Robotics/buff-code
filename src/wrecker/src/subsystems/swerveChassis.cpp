#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"
#include "swerveChassis.h"

#include "algorithms/PID_Filter.h"

SwerveChassis::SwerveChassis() {

}


void SwerveChassis::setup(C_SwerveChassis* data, S_Robot* r_state) {

  config = data;
  state = r_state;

  // // Init modules
  moduleFR.setup(&data->FR, state, &state->chassis.FR);
  moduleFL.setup(&data->FL, state, &state->chassis.FL);
  moduleBL.setup(&data->RL, state, &state->chassis.RL);
  moduleBR.setup(&data->RR, state, &state->chassis.RR);

  float baseRadius = sqrt(pow(config->baseLength, 2) + pow(config->baseWidth, 2));
  drivebaseConstant = config->baseLength / baseRadius;

  calibrate();
}

void SwerveChassis::update(unsigned long deltaTime) {

  // float driveX = (state->driverInput.w - state->driverInput.a) * cos(state->gimbal.yaw);
  // float driveY = (state->driverInput.w - state->driverInput.a) * sin(state->gimbal.yaw);

  float js1 = state->driverInput.leftStickX;
  float js2 = state->driverInput.leftStickY;
  js1 = map(js1, 364, 1684, 0, 1000) / 1000.0;
  js2 = map(js2, 364, 1684, 0, 1000) / 1000.0;

  drive(js1, 0, 0, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, unsigned long deltaTime) {

  float gimbalAngle = 0;

  float newDriveX = -driveX * cos(radiansToDegrees(gimbalAngle)) + -driveY * sin(radiansToDegrees(-gimbalAngle));
  float newDriveY = -driveX * sin(radiansToDegrees(gimbalAngle)) + -driveY * cos(radiansToDegrees(gimbalAngle));

  float A = newDriveX - spin * drivebaseConstant;
  float B = newDriveX + spin * drivebaseConstant;
  float C = newDriveY - spin * drivebaseConstant;
  float D = newDriveY + spin * drivebaseConstant;

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

