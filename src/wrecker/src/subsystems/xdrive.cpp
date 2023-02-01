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
  
  this->driveMotors[0].init(3, 2); // FR
  this->driveMotors[1].init(1, 2); // FL
  this->driveMotors[2].init(6, 2); // BL
  this->driveMotors[3].init(5, 2); // BR
}

void XDrive::update(float deltaTime) {
  for (c620CAN driveMotor : this->driveMotors) driveMotor.updateMotor();

  float x = ((state->driverInput.rightStickX - 364) / 1320.0) * 2 - 1;
  float y = ((state->driverInput.leftStickX - 364) / 1320.0) * 2 - 1;
  float spin = -1 * (((state->driverInput.remoteWheel - 364) / 1320.0) * 2 - 1);

  int max_rpm = 9000;

  double speed1 = y - x - spin;
  double speed2 = y + x + spin;
  double speed3 = y - x + spin;
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

  float power = state->refSystem.chassis_current * state->refSystem.chassis_voltage / 1000000.0;
  float limit = state->refSystem.robot_power_lim;

  // float k = (power / limit) * 4;
  // if (k < 1) k = 1;

  // Serial.print(", ");
  // Serial.print(k);
  // Serial.print(", ");

  float output[] = {0, 0, 0, 0};

  // PID
  state->chassis.FR.driveVel.R = speed1 * max_rpm;
  PID_Filter(&config->swerveChassis.FR.driveVel, &state->chassis.FR.driveVel, this->driveMotors[0].getRpm(), deltaTime);
  output[0] = state->chassis.FR.driveVel.Y;

  state->chassis.FL.driveVel.R = -speed2 * max_rpm;
  PID_Filter(&config->swerveChassis.FL.driveVel, &state->chassis.FL.driveVel, this->driveMotors[1].getRpm(), deltaTime);
  output[1] = state->chassis.FL.driveVel.Y;

  state->chassis.RL.driveVel.R = -speed3 * max_rpm;
  PID_Filter(&config->swerveChassis.RL.driveVel, &state->chassis.RL.driveVel, this->driveMotors[2].getRpm(), deltaTime);
  output[2] = state->chassis.RL.driveVel.Y;

  state->chassis.RR.driveVel.R = speed4 * max_rpm;
  PID_Filter(&config->swerveChassis.RR.driveVel, &state->chassis.RR.driveVel, this->driveMotors[3].getRpm(), deltaTime);
  output[3] = state->chassis.RL.driveVel.Y;

  float sum = 0;
  for (int i = 0; i < 4; i++) {
    if (output[i] > 1) output[i] = 1;
    if (output[i] < -1) output[i] = -1;
    sum += fabs(output[i]) * 20 * state->refSystem.chassis_voltage / 1000.0;
  }

  float k = sum / limit;
  float k2 = power / limit;
  if (k2 < 1) k = 1;
  Serial.print(sum);
  Serial.print(", ");
  Serial.print(limit);
  Serial.print(", ");
  Serial.print(k);
  Serial.print(", ");
  Serial.print(power);

  //for (int i = 0; i < 4; i++) output[i] /= k;

  float sum2 = 0;
  for (int i = 0; i < 4; i++) {
    sum2 += fabs(output[i]) * 20 * state->refSystem.chassis_voltage / 1000.0;
  }

  Serial.print(", ");
  Serial.println(sum2);


  this->driveMotors[0].setPower(output[0]);
  this->driveMotors[1].setPower(output[1]);
  this->driveMotors[2].setPower(output[2]);
  this->driveMotors[3].setPower(output[3]);

  // int poww = 1000;
  // this->driveMotors[0].setPower(poww);
  // this->driveMotors[1].setPower(poww);
  // this->driveMotors[2].setPower(poww);
  // this->driveMotors[3].setPower(poww);
  // Serial.println(poww);
}