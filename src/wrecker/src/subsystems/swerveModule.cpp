#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"

#include "algorithms/PID_Filter.h"

SwerveModule::SwerveModule()
{
}

void SwerveModule::setup(C_SwerveModule *data, S_Robot *r_state, S_SwerveModule *modState)
{
  config = data;
  state = r_state;

  moduleState = modState;

  this->steerMotor.init(config->steerMotorID, 1, config->steerEncoderID);
  if (config->cornerID == 2)
    this->driveMotor.init(config->driveMotorID, 2);
  else
    this->driveMotor.init(config->driveMotorID, 1);
}

void SwerveModule::calibrate()
{
  this->steerMotor.updateMotor();
  this->steerOffset = findCalibrationMatch(this->steerMotor.getAngle(), this->config->alignment, 9);
  this->steerRollover = 0;
  calibrated = true;
  rampedSpeed = 0;
}

void SwerveModule::update(float speed, float angle, float deltaTime)
{
  calibrated = true;
  // Convert sensor and input units
  float inputAngle = angle + config->absolute_offset;
  if (speed == 0)
  {
    inputAngle = prevSteerAngle;
  }
  if (inputAngle < 0)
    inputAngle += 360;

  float rawSteerAngle = this->steerMotor.getAngle();
  if (calibrated)
  {
    float steeringDifference = this->prevRawSteerAngle - rawSteerAngle;
    if (steeringDifference < -180)
      this->steerRollover--;
    else if (steeringDifference > 180)
      this->steerRollover++;
  }
  this->prevRawSteerAngle = rawSteerAngle;
  float steerAngle = realizeSteerAngle(rawSteerAngle);
  this->prevSteerAngle = steerAngle;

  float rpm = steerMotor.getRpm() / 100.0; // The number 100 is based on the gear ratio of the motor and the steer belt

  // Inversion logic
  // Decides wether or not to switch the wheel's drive direction
  int inversion = 1;
  float error = inputAngle - steerAngle;
  float shadow = error - 360.0;
  if (fabs(shadow) < error)
    error = shadow;
  if (abs(error) > 90)
  {
    inversion = -1;
    inputAngle -= 180;
    if (inputAngle < 0)
      inputAngle += 360;
  }

  if (abs(speed) > 0)
  {
    this->prevInversion = inversion;
  }
  else
  {
    inversion = this->prevInversion;
  }

  speed = speed * inversion;

  // Speed ramping
  float rampLimit = this->config->rampLimit;
  if (this->state->refSystem.robot_level == 3 || this->state->driverInput.s2 == 2)
  {
    rampLimit = this->config->rampLimitHigh;
  }

  if (rampedSpeed < speed)
  {
    rampedSpeed += (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed > speed)
    {
      rampedSpeed = speed;
    }
  }
  else if (rampedSpeed > speed)
  {
    rampedSpeed -= (deltaTime / 1000000.0) / this->config->rampLimit;
    if (rampedSpeed < speed)
    {
      rampedSpeed = speed;
    }
  }

  // Ref limiting
  if (state->robot == 3 && state->driverInput.s2 == 2)
  {
    // 1v1
    state->chassis.maxRpm = 4000;
  }
  else
  {
    // 3v3
    switch (state->refSystem.robot_level)
    {
    case 1:
      state->chassis.maxRpm = 2800;
      break;
    case 2:
      state->chassis.maxRpm = 3600;
      break;
    case 3:
      state->chassis.maxRpm = 4000;
      break;
    default:
      state->chassis.maxRpm = 2800;
      break;
    }
  }

  // Steer Velocity PID
  // Serial.println(driveMotor.getRpm());

  config->steerPos.continuous = true;
  tmp_steerPos.R = inputAngle;
  PID_Filter(&config->steerPos, &tmp_steerPos, steerAngle, deltaTime);

  // Steer Position PID
  tmp_steerVel.R = -tmp_steerPos.Y;
  PID_Filter(&config->steerVel, &tmp_steerVel, rpm, deltaTime);

  if (tmp_steerVel.Y > 1.0)
    tmp_steerVel.Y = 1.0;
  if (tmp_steerVel.Y < -1.0)
    tmp_steerVel.Y = -1.0;

  // Drive Velocity PID
  float jsi1 = ((state->driverInput.rightStickX - 364) / 1320.0);
  float jsi2 = ((state->driverInput.leftStickX - 364) / 1320.0);

  if (config->cornerID == 2) {
    float pitch_angle = driveMotor.getAngle();

    tmp_steerPos.R = jsi1*295+25;

    config->steerPos.K[0] = 5;//30; // P = 5
    config->steerPos.K[2] = 10;//1800; // D = 10
    PID_Filter(&config->steerPos, &tmp_steerPos, pitch_angle, deltaTime);
    moduleState->driveVel.R = tmp_steerPos.Y;
    PID_Filter(&config->driveVel, &moduleState->driveVel, driveMotor.getRpm(), deltaTime);
  } else {
    // if (state->driverInput.s2 == 1) {
    //   if (config->cornerID == 1)
    //     moduleState->driveVel.R = -9000;
    //   else
    //     moduleState->driveVel.R = 9000;
    // } else
    //   moduleState->driveVel.R = 0;
    // //moduleState->driveVel.R = ((jsi2 * 2) - 1) * 100;//tmp_steerPos.Y;
    // PID_Filter(&config->driveVel, &moduleState->driveVel, driveMotor.getRpm(), deltaTime);
  }

  // if (state->driverInput.s2 == 1) {
  //   if (config->cornerID == 1)
  //     moduleState->driveVel.R = -9000;
  //   else
  //     moduleState->driveVel.R = 9000;
  // } else
  //   moduleState->driveVel.R = 0;
  if (config->cornerID == 2) {
    if (state->driverInput.s2 == 1)
      moduleState->driveVel.R = 9000;
    else
      moduleState->driveVel.R = 0;
  } else
    moduleState->driveVel.R = ((jsi2 * 2) - 1) * 4000;//tmp_steerPos.Y;
  PID_Filter(&config->driveVel, &moduleState->driveVel, driveMotor.getRpm(), deltaTime);

  Serial.print(driveMotor.getRpm());
  Serial.print(" ");
  driveMotor.updateMotor();

  // Set motor power
  driveMotor.setPower(moduleState->driveVel.Y);
}

int SwerveModule::findCalibrationMatch(int currValue, int *alignmentTable, int tableSize)
{
  int smallestDistance = 360;
  int bestOffset = 0;
  for (int i = 0; i < tableSize; i++)
  {
    int distance = abs(currValue - alignmentTable[i]);
    int shadowDistance = (currValue - alignmentTable[i]) + 360;
    if (shadowDistance < distance)
      distance = shadowDistance;
    if (distance < smallestDistance)
    {
      smallestDistance = distance;
      bestOffset = alignmentTable[i];
    }
  }
  return bestOffset;
}

int SwerveModule::getSteerId()
{
  return config->steerMotorID;
}

float SwerveModule::realizeSteerAngle(float rawSteerAngle)
{
  float steerAngle = ((rawSteerAngle - this->steerOffset + (this->steerRollover * 360)) * (9.0 / 25.0));

  steerAngle = fmod(steerAngle, 360.0);
  if (steerAngle < 0)
    steerAngle += 360;
  return steerAngle;
}