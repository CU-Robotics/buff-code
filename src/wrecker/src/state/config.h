#include "Arduino.h"

#ifndef CONFIG_H
#define CONFIG_H

struct C_PID
{
  //  Kp, Ki, Kd
  float K[3] = {0.5, 0.0, 0.25};

  // Integrator low, high
  float Imin = -0.1f;
  float Imax = 0.1f;

  // Output low, high
  float Ymin = -180.0f;
  float Ymax = 180.0f;

  // Continuous output
  bool continuous = false;
};

struct C_Teensy {
  int loopStall = 1000; // microseconds
};

struct C_SwerveModule {
  //  Unique module Identifiers
  int moduleID = -1;
  int cornerID = -1;
  int steerMotorID = -1;
  int driveMotorID = -1;
  int steerEncoderID = -1;

  //  Alignment data ???
  int alignment[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
  int absolute_offset = 0;

  // PIDs
  C_PID steerVel;
  C_PID steerPos;
  C_PID driveVel;

  float rampLimit = 0.01;
};

struct C_SwerveChassis {

  float baseWidth = 14.5;
  float baseLength = 14.5;

  //  Current limit for level: lvl0, lvl1, lvl2, lvl3
  float currentLimit[4] = {40.0 / 24.0, 60.0 / 24.0, 80.0 / 24.0, 100.0 / 24.0};

  //  SwerveModules
  C_SwerveModule FR;
  C_SwerveModule FL;
  C_SwerveModule RL;
  C_SwerveModule RR;
};

struct C_RailChassis {
  // Stopping nodes (UNIMPLEMENTED)
  float nodes[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // PIDs
  C_PID drivePos;
  C_PID driveVel;
};

struct C_Gimbal {
  float sensitivity = 0.000005;

  // Angle offset for motors: yaw, pitch
  float yawOffset = 90.0;
  float pitchOffset = 12.0;

  // Pitch softstops
  float pitchMin = 0;
  float pitchMax = 75;

  // Motor IDs
  float pitchMotorID = 1;
  float yawMotorID = 2;

  C_PID yawVel;
  C_PID yawPos;
  C_PID pitchVel;
  C_PID pitchPos;
};

// Configured for cooling focus by default
struct C_Shooter17 {
  //  Feeder RPM for: low, high, burst
  float feedRPMLow = 10.0;
  float feedRPMHigh = 25.0;
  float feedRPMBurst = 50.0;

  //  Flywheel power for lvl: lvl0, lvl1, lvl2, lvl3
  float flywheelPower[4] = {0.15, 0.17, 0.2, 0.22};

  C_PID feedPID;
};

struct C_Shooter42 {
  //  Feeder timeout
  float feedTimeout = 0.5; // Minimum time between consecutive shots, in seconds

  //  Flywheel power for lvl: lvl0, lvl1, lvl2, lvl3
  float flywheelPower[4] = {0.15, 0.17, 0.2, 0.22};
};

struct C_Robot {
  C_Gimbal gimbal;
  C_Shooter17 shooter17;
  C_Shooter42 shooter42;
  C_RailChassis railChassis;
  C_SwerveChassis swerveChassis;
};

#endif