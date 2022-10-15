#include <Arduino.h>

#include <vector>

#include "xdrive.h"

#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "algorithms/PID_Filter.h"

XDrive::XDrive(C_Robot *r_config, S_Robot *r_state) {
  config = r_config;
  state = r_state;
  
  this->driveMotors[0].init(3, 2);
  this->driveMotors[1].init(7, 2);
  this->driveMotors[2].init(1, 2);
  this->driveMotors[3].init(6, 2);
}

void XDrive::update(float deltaTime) {
  for (c620CAN driveMotor : this->driveMotors) driveMotor.updateMotor();

  float x = ((state->driverInput.rightStickX - 364) / 1320.0) * 2 - 1;
  float y = ((state->driverInput.leftStickX - 364) / 1320.0) * 2 - 1;
  float spin = -1 * (((state->driverInput.remoteWheel - 364) / 1320.0) * 2 - 1);

  // Serial.print(x);
  // Serial.print(" - ");
  // Serial.print(y);
  // Serial.print(" - ");
  // Serial.print(spin, BIN);
  // Serial.println();

  int max_rpm = 9000;

  double speed2 = y + x + spin;
  double speed3 = y - x + spin;
  double speed1 = y - x - spin;
  double speed4 = y + x - spin;

  if (isnan(speed1)) speed1 = 0;
  if (isnan(speed2)) speed2 = 0;
  if (isnan(speed3)) speed3 = 0;
  if (isnan(speed4)) speed4 = 0;

  double bullshit = abs(speed1 * 100);
  if (abs(speed2 * 100) > bullshit) bullshit = abs(speed2 * 100);
  if (abs(speed3 * 100) > bullshit) bullshit = abs(speed3 * 100);
  if (abs(speed4 * 100) > bullshit) bullshit = abs(speed4 * 100);
  bullshit /= 100;

  if (bullshit > 1.0) {
    speed1 = speed1 / bullshit;
    speed2 = speed2 / bullshit;
    speed3 = speed3 / bullshit;
    speed4 = speed4 / bullshit;
  }

  Serial.print(speed1);
  Serial.print(" - ");
  Serial.print(speed2);
  Serial.println();

  // PID
  state->chassis.FR.driveVel.R = speed1 * max_rpm;
  PID_Filter(&config->swerveChassis.FR.driveVel, &state->chassis.FR.driveVel, this->driveMotors[0].getRpm(), deltaTime);
  this->driveMotors[0].setPower(state->chassis.FR.driveVel.Y);

  state->chassis.FL.driveVel.R = -speed2 * max_rpm;
  PID_Filter(&config->swerveChassis.FL.driveVel, &state->chassis.FL.driveVel, this->driveMotors[1].getRpm(), deltaTime);
  this->driveMotors[1].setPower(state->chassis.FL.driveVel.Y);

  state->chassis.RL.driveVel.R = -speed3 * max_rpm;
  PID_Filter(&config->swerveChassis.RL.driveVel, &state->chassis.RL.driveVel, this->driveMotors[2].getRpm(), deltaTime);
  this->driveMotors[2].setPower(state->chassis.RL.driveVel.Y);

  state->chassis.RR.driveVel.R = speed4 * max_rpm;
  PID_Filter(&config->swerveChassis.RR.driveVel, &state->chassis.RR.driveVel, this->driveMotors[3].getRpm(), deltaTime);
  this->driveMotors[3].setPower(state->chassis.RR.driveVel.Y);
}