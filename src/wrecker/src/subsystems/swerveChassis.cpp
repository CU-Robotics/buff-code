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

  int bl_alignment[9] = {23, 63, 102, 142, 182, 222, 261, 301, 340};
  int br_alignment[9] = {18, 60, 99, 139, 179, 220, 260, 299, 341};
  int fr_alignment[9] = {20, 60, 100, 140, 181, 221, 261, 302, 341};
  int fl_alignment[9] = {2, 42, 85, 125, 166, 206, 246, 288, 327};

  // FRONT RIGHT
  data->FR.cornerID = 0;
  data->FR.steerMotorID = 4;
  data->FR.steerEncoderID = 5;
  data->FR.driveMotorID = 5;
  data->FR.absolute_offset = 45;
  for (int i = 0; i < 9; i++)
  {
    data->FR.alignment[i] = fr_alignment[i];
  }
  data->FR.steerVel.K[0] = 0.07;
  data->FR.steerVel.K[1] = 0;
  data->FR.steerVel.K[2] = 0;
  data->FR.steerPos.K[0] = 1.2;
  data->FR.steerPos.K[1] = 0;
  data->FR.steerPos.K[2] = 0;
  data->FR.driveVel.K[0] = 0.0006;

  // FRONT LEFT
  data->FL.cornerID = 1;
  data->FL.steerMotorID = 3;
  data->FL.steerEncoderID = 2;
  data->FL.driveMotorID = 6;
  data->FL.absolute_offset = -45;
  for (int i = 0; i < 9; i++)
  {
    data->FL.alignment[i] = fl_alignment[i];
  }
  data->FL.steerVel.K[0] = 0.07;
  data->FL.steerVel.K[1] = 0;
  data->FL.steerVel.K[2] = 0;
  data->FL.steerPos.K[0] = 1.2;
  data->FL.steerPos.K[1] = 0;
  data->FL.steerPos.K[2] = 0;
  data->FL.driveVel.K[0] = 0.0006;

  // BACK LEFT
  data->RL.cornerID = 2;
  data->RL.steerMotorID = 2;
  data->RL.steerEncoderID = 3;
  data->RL.driveMotorID = 7;
  data->RL.absolute_offset = -135;
  for (int i = 0; i < 9; i++)
  {
    data->RL.alignment[i] = bl_alignment[i];
  }
  data->RL.steerVel.K[0] = 0.07;
  data->RL.steerVel.K[1] = 0;
  data->RL.steerVel.K[2] = 0;
  data->RL.steerPos.K[0] = 1.2;
  data->RL.steerPos.K[1] = 0;
  data->RL.steerPos.K[2] = 0;
  data->RL.driveVel.K[0] = 0.0006;

  // BACK RIGHT
  data->RR.cornerID = 3;
  data->RR.steerMotorID = 1;
  data->RR.steerEncoderID = 4;
  data->RR.driveMotorID = 8;
  data->RR.absolute_offset = 135;
  for (int i = 0; i < 9; i++)
  {
    data->RR.alignment[i] = br_alignment[i];
  }
  data->RR.steerVel.K[0] = 0.07;
  data->RR.steerVel.K[1] = 0;
  data->RR.steerVel.K[2] = 0;
  data->RR.steerPos.K[0] = 1.2;
  data->RR.steerPos.K[1] = 0;
  data->RR.steerPos.K[2] = 0;
  data->RR.driveVel.K[0] =  0.0006;

  // // Init modules
  moduleFR.setup(&data->FR, state, &state->chassis.FR);
  moduleFL.setup(&data->FL, state, &state->chassis.FL);
  moduleBL.setup(&data->RL, state, &state->chassis.RL);
  moduleBR.setup(&data->RR, state, &state->chassis.RR);

  float baseRadius = sqrt(pow(config->baseLength, 2) + pow(config->baseWidth, 2));
  drivebaseConstant = config->baseLength / baseRadius;

  //calibrate();
}

void SwerveChassis::update(unsigned long deltaTime) {
  if (state->driverInput.f && !calibrated) {
    calibrate();
    calibrated = true;
    Serial.println("Calibrate");
  }

  // float driveX = (state->driverInput.w - state->driverInput.a) * cos(state->gimbal.yaw);
  // float driveY = (state->driverInput.w - state->driverInput.a) * sin(state->gimbal.yaw);

  float js1 = state->driverInput.leftStickX;
  float js2 = state->driverInput.leftStickY;
  js1 = map(js1, 364, 1684, 0, 1000) / 1000.0;
  js2 = map(js2, 364, 1684, 0, 1000) / 1000.0;

  int x = state->driverInput.w - state->driverInput.s;
  int y = state->driverInput.d - state->driverInput.a;
  int s = state->driverInput.q - state->driverInput.e;

  drive(x, y, s, deltaTime);
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
  // moduleFL.update(speedFL, angleFL, deltaTime);
  // moduleBL.update(speedBL, angleBL, deltaTime);
  // moduleBR.update(speedBR, angleBR, deltaTime);
}

float SwerveChassis::radiansToDegrees(float radians) {
  double degrees;
  degrees = radians * 180 / PI;
  return degrees;
}