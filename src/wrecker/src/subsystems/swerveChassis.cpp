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

  // Configure PIDs
  data->FR.steerVel.K[0] = 0.03;
  data->FR.steerPos.K[0] = 1.8;
  data->FR.steerPos.K[2] = 10.0;
  data->FR.steerPos.Ymax = 1000;
  data->FR.steerPos.Ymin = -1000;
  data->FR.driveVel.K[0] = 0.0006;
  data->FR.driveVel.K[2] = 0;


  data->FL.steerVel.K[0] = data->FR.steerVel.K[0];
  data->FL.steerPos.K[0] = data->FR.steerPos.K[0];
  data->FL.steerPos.K[2] = data->FR.steerPos.K[2];
  data->FL.steerPos.Ymax = data->FR.steerPos.Ymax;
  data->FL.steerPos.Ymin = data->FR.steerPos.Ymin;
  data->FL.driveVel.K[0] = data->FR.driveVel.K[0];
  data->FL.driveVel.K[2] = data->FR.driveVel.K[2];

  data->RL.steerVel.K[0] = data->FR.steerVel.K[0];
  data->RL.steerPos.K[0] = data->FR.steerPos.K[0];
  data->RL.steerPos.K[2] = data->FR.steerPos.K[2];
  data->RL.steerPos.Ymax = data->FR.steerPos.Ymax;
  data->RL.steerPos.Ymin = data->FR.steerPos.Ymin;
  data->RL.driveVel.K[0] = data->FR.driveVel.K[0];
  data->RL.driveVel.K[2] = data->FR.driveVel.K[2];

  data->RR.steerVel.K[0] = data->FR.steerVel.K[0];
  data->RR.steerPos.K[0] = data->FR.steerPos.K[0];
  data->RR.steerPos.K[2] = data->FR.steerPos.K[2];
  data->RR.steerPos.Ymax = data->FR.steerPos.Ymax;
  data->RR.steerPos.Ymin = data->FR.steerPos.Ymin;
  data->RR.driveVel.K[0] = data->FR.driveVel.K[0];
  data->RR.driveVel.K[2] = data->FR.driveVel.K[2];


  // Init modules
  moduleFR.setup(&data->FR, state, &state->chassis.FR);
  moduleFL.setup(&data->FL, state, &state->chassis.FL);
  moduleBL.setup(&data->RL, state, &state->chassis.RL);
  moduleBR.setup(&data->RR, state, &state->chassis.RR);

  float baseRadius = sqrt(pow(config->baseLength, 2) + pow(config->baseWidth, 2));
  drivebaseConstant = config->baseLength / baseRadius;
}

void SwerveChassis::update(unsigned long deltaTime) {
  if (state->driverInput.b && !calibrated) {
    calibrated = true;
    calibrate();
    Serial.println("SwerveChassis Calibrated");
  }

  int x = state->driverInput.d - state->driverInput.a;
  int y = state->driverInput.s - state->driverInput.w;
  int s = state->driverInput.z - state->driverInput.x;

  if (this->state->driverInput.shift && !shiftPressed) {
    if (this->state->chassis.beyblade) {
      this->state->chassis.beyblade = false;
    } else {
      this->state->chassis.beyblade = true;
    }
    shiftPressed = true;
  } else if (!this->state->driverInput.shift) {
    shiftPressed = false;
  }

  if (this->state->chassis.beyblade) {
    s = 1.0;
  }

  drive(x, y, s, deltaTime);

  if (s == 0) {
    this->state->chassis.spin = 1.0;
  } else if (s != 0 && x != 0) {
    if (y != 0) {
      this->state->chassis.spin = 1.0;
    } else {
      this->state->chassis.spin = 1.0;
    }
  } else if (s != 0 && y != 0) {
    this->state->chassis.spin = 1.0;
  } else {
    if (state->robot == 3) {
      switch(this->state->refSystem.robot_level) {
        case 1:
          this->state->chassis.spin = 1;
          break;
        case 2:
          this->state->chassis.spin = 1.1;
          break;
        case 3:
          this->state->chassis.spin = 1.4;
          break;
        default:
          this->state->chassis.spin = 1;
          break;
      }
    } else if (state->robot == 1) {
      switch(this->state->refSystem.robot_level) {
        case 1:
          this->state->chassis.spin = 1;
          break;
        case 2:
          this->state->chassis.spin = 1;
          break;
        case 3:
          this->state->chassis.spin = 1;
          break;
        default:
          this->state->chassis.spin = 1;
          break;
      }
    }
  }
}

void SwerveChassis::calibrate() {
  moduleFR.calibrate();
  moduleFL.calibrate();
  moduleBL.calibrate();
  moduleBR.calibrate();
}

void SwerveChassis::drive(float driveX, float driveY, float spin, unsigned long deltaTime) {
  float gimbalAngle = this->state->gimbal.yawGlobal;

  // Apply rotation matrix so that drive inputs are gimbal-relative
  float newDriveX = -driveX * cos(degreesToRadians(gimbalAngle)) + -driveY * sin(degreesToRadians(-gimbalAngle));
  float newDriveY = -driveX * sin(degreesToRadians(gimbalAngle)) + -driveY * cos(degreesToRadians(gimbalAngle));

  // Swerve math
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

  // Update each module
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

float SwerveChassis::degreesToRadians(float degrees) {
  double radians;
  radians = degrees * (PI / 180.0);
  return radians;
}