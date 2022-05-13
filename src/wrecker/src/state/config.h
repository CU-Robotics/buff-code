#include "Arduino.h"

#ifndef CONFIG_H
#define CONFIG_H

struct C_PID
{
  float K[3] = {0.5f, 0.5f, 0.25f};

  float Imin = -45.0f;
  float Imax = 45.0f;

  float Ymin = -180.0f;
  float Ymax = 180.0f;

  bool continuous = false;
};

struct C_Teensy {
  int loopStall = 1000; // microseconds
};

struct C_SwerveModule {
  int moduleID;
  int alignment[9];

  C_PID steerVel;
  C_PID steerPos;
  C_PID driveVel;
};

struct C_SwerveChassis {
  float drivebaseWidth = 14.5;
  float drivebaseLength = 14.5;

  float currentLimitLvl0 = 40.0 / 24.0;
  float currentLimitLvl1 = 60.0 / 24.0;
  float currentLimitLvl2 = 80.0 / 24.0;
  float currentLimitLvl3 = 100.0 / 24.0;

  C_SwerveModule FR;
  C_SwerveModule FL;
  C_SwerveModule RL;
  C_SwerveModule RR;
};

struct C_RailChassis {
  int numNodes = 10;
  float nodes[10];

  C_PID drivePos;
  C_PID driveVel;
};

struct C_Gimbal {
  float sensitivity = 1.0;

  float pitchOffset = 12;
  float yawOffset = 90;

  C_PID pitch_PID;
  C_PID yaw_PID;
};

// Configured for cooling focus by default
struct C_Shooter17 {
  float feedRPMLow = 10;
  float feedRPMHigh = 25;
  float feedRPMBurst = 50;

  float flywheelPowerLvl0 = 0.15;
  float flywheelPowerLvl1 = 0.17;
  float flywheelPowerLvl2 = 0.2;
  float flywheelPowerLvl3 = 0.22;
};

struct C_Shooter42 {
  float feedTimeout = 0.5; // Minimum time between consecutive shots, in seconds

  float flywheelPowerLvl0 = 0.15;
  float flywheelPowerLvl1 = 0.17;
  float flywheelPowerLvl2 = 0.2;
  float flywheelPowerLvl3 = 0.22;
};

struct C_Robot {
  C_Gimbal gimbal;
  C_Shooter17 shooter17;
  C_Shooter42 shooter42;
  C_RailChassis railChassis;
  C_SwerveChassis swerveChassis;
};

#endif