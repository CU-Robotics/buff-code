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

void PID_serial_event(C_PID*);
void dump_PID_config(C_PID*, char*);

struct C_Teensy {
  int loopStall = 1000; // microseconds
};

struct C_SwerveModule {
  short moduleID;
  int alignment[9];

  C_PID steerVel;
  C_PID steerPos;
  C_PID driveVel;
};

void SwerveModule_serial_event(C_SwerveModule*);

struct C_SwerveChassis {
  float drivebaseWidth = 14.5;
  float drivebaseLength = 14.5;

  float currentLimitLvl0 = 40.0 / 24.0;
  float currentLimitLvl1 = 60.0 / 24.0;
  float currentLimitLvl2 = 80.0 / 24.0;
  float currentLimitLvl3 = 100.0 / 24.0;

  C_SwerveModule moduleFR;
  C_SwerveModule moduleFL;
  C_SwerveModule moduleBL;
  C_SwerveModule moduleBR;
};

void SwerveChassis_serial_event(C_SwerveChassis*);


struct C_RailChassis {
  int numNodes = 10;
  float nodes[10];

  C_PID driveVel;
  C_PID drivePos;
};

void RailChassis_serial_event(C_RailChassis*);


struct C_Gimbal {
  float sensitivity = 1.0;

  float pitchOffset = 12;
  float yawOffset = 90;

  C_PID pitch_PID;
  C_PID yaw_PID;
};

void Gimbal_serial_event(C_Gimbal*);

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

void Shooter17_serial_event(C_Shooter17*);

struct C_Shooter42 {
  float feedTimeout = 0.5; // Minimum time between consecutive shots, in seconds

  float flywheelPowerLvl0 = 0.15;
  float flywheelPowerLvl1 = 0.17;
  float flywheelPowerLvl2 = 0.2;
  float flywheelPowerLvl3 = 0.22;
};

void Shooter42_serial_event(C_Shooter42*);

#endif