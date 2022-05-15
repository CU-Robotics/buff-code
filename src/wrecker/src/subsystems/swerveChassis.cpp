#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"
#include "swerveChassis.h"

#include "algorithms/PID_Filter.h"

SwerveChassis::SwerveChassis() {

}

void SwerveChassis::setup(C_SwerveChassis *data, S_Robot *r_state) {
  int bl_alignment[9] = {18, 60, 99, 139, 179, 220, 260, 299, 341};
  int br_alignment[9] = {20, 60, 100, 140, 181, 221, 261, 302, 341};
  int fr_alignment[9] = {23, 63, 102, 142, 182, 222, 261, 301, 340};
  int fl_alignment[9] = {2, 42, 85, 125, 166, 206, 246, 288, 327};

  // FRONT RIGHT
  C_SwerveModule FR_Config;
  FR_Config.moduleID = 4;
  for (int i = 0; i < 9; i++)
  {
    FR_Config.alignment[i] = fr_alignment[i];
  }
  FR_Config.steerVel.K[0] = 0;
  FR_Config.steerVel.K[1] = 0;
  FR_Config.steerVel.K[2] = 0;
  FR_Config.steerPos.K[0] = 0;
  FR_Config.steerPos.K[1] = 0;
  FR_Config.steerPos.K[2] = 0;

  // FRONT LEFT
  C_SwerveModule FL_Config;
  FL_Config.moduleID = 3;
  // FL_Config.alignment = {2, 42, 85, 125, 166, 206, 246, 288, 327};
  for (int i = 0; i < 9; i++)
  {
    FL_Config.alignment[i] = fl_alignment[i];
  }
  FL_Config.steerVel.K[0] = 10;
  FL_Config.steerVel.K[1] = 0;
  FL_Config.steerVel.K[2] = 0;
  FL_Config.steerPos.K[0] = 0;
  FL_Config.steerPos.K[1] = 0;
  FL_Config.steerPos.K[2] = 0;

  // BACK LEFT
  C_SwerveModule BL_Config;
  BL_Config.moduleID = 2;
  // BL_Config.alignment = {18, 60, 99, 139, 179, 220, 260, 299, 341};
  for (int i = 0; i < 9; i++)
  {
    BL_Config.alignment[i] = bl_alignment[i];
  }
  BL_Config.steerVel.K[0] = 0;
  BL_Config.steerVel.K[1] = 0;
  BL_Config.steerVel.K[2] = 0;
  BL_Config.steerPos.K[0] = 0;
  BL_Config.steerPos.K[1] = 0;
  BL_Config.steerPos.K[2] = 0;

  // BACK RIGHT
  C_SwerveModule BR_Config;
  BR_Config.moduleID = 1;
  // BR_Config.alignment = {20, 60, 100, 140, 181, 221, 261, 302, 341};
  for (int i = 0; i < 9; i++)
  {
    BR_Config.alignment[i] = br_alignment[i];
  }
  BR_Config.steerVel.K[0] = 0;
  BR_Config.steerVel.K[1] = 0;
  BR_Config.steerVel.K[2] = 0;
  BR_Config.steerPos.K[0] = 0;
  BR_Config.steerPos.K[1] = 0;
  BR_Config.steerPos.K[2] = 0;

  // Init modules
  moduleFR.setup(&FR_Config, state);
  moduleFL.setup(&FL_Config, state);
  moduleBL.setup(&BL_Config, state);
  moduleBR.setup(&BR_Config, state);

  // Normal Setup
  config = data;
  state = r_state;

  float drivebaseRadius = sqrt(pow(config->drivebaseLength, 2) + pow(this->config->drivebaseWidth, 2));
  drivebaseConstant = config->drivebaseLength / drivebaseRadius;

  calibrate();
}

void SwerveChassis::update(float deltaTime) {
  float driveX = (state->driverInput.w - state->driverInput.a) * cos(state->gimbal.yaw);
  float driveY = (state->driverInput.w - state->driverInput.a) * sin(state->gimbal.yaw);

  float js1 = this->state->driverInput.leftStickX;
  float js2 = this->state->driverInput.leftStickY;
  js1 = map(js1, 364, 1684, 0, 1000) / 1000.0;
  js2 = map(js2, 364, 1684, 0, 1000) / 1000.0;

  // Serial.println("looking for js");
  // Serial.print(js1);
  // Serial.print(" - ");
  // Serial.println(js2);
  drive(js1, 0, 0, deltaTime);
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {

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

  float angleFR = this->radiansToDegrees(atan2(B, D));
  float angleFL = this->radiansToDegrees(atan2(B, C));
  float angleBL = this->radiansToDegrees(atan2(A, C));
  float angleBR = this->radiansToDegrees(atan2(A, D));

  this->moduleFR.update(speedFR, angleFR, deltaTime);
  //this->moduleFL.update(speedFL, angleFL, deltaTime);
  //this->moduleBL.update(speedBL, angleBL, deltaTime);
  //this->moduleBR.update(speedBR, angleBR, deltaTime);

  // Serial.print(angleFR);
  // Serial.print(" - ");
  // Serial.print(angleFL);
  // Serial.print(" - ");
  // Serial.print(angleBL);
  // Serial.print(" - ");
  // Serial.print(angleBR);
  // Serial.println(" - ");
}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}

