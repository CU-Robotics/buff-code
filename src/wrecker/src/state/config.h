#define CONFIG_H

struct C_Config {

};

struct C_Teensy: C_Config {
  int loopStall = 1000; // microseconds
};

struct C_SwerveModule: C_Config {
  short moduleID;

  int alignment[9];

  float steerVelP;
  float steerVelI;
  float steerVelD;
  float steerVelF;

  float steerPosP;
  float steerPosI;
  float steerPosD;
  float steerPosF;

  float driveVelP;
  float driveVelI;
  float driveVelD;
  float driveVelF;
};

struct C_SwerveChassis: C_Config {
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

struct C_RailChassis: C_Config {
  float VelP;
  float VelI;
  float VelD;
  float VelF;

  float PosP;
  float PosI;
  float PosD;
  float PosF;

  float* nodes;
  int numNodes;
};

struct C_Gimbal: C_Config {
  float sensitivity = 1.0;

  float pitchOffset = 12;
  float yawOffset = 90;

  float maxPitch;
  float minPitch;

  float pitchP;
  float pitchI;
  float pitchD;
  float pitchF;

  float yawP;
  float yawI;
  float yawD;
  float yawF;
};

// Configured for cooling focus by default
struct C_17mmShooter: C_Config {
  float feedRPMLow = 10;
  float feedRPMHigh = 25;
  float feedRPMBurst = 50;

  float flywheelPowerLvl0 = 0.15;
  float flywheelPowerLvl1 = 0.17;
  float flywheelPowerLvl2 = 0.2;
  float flywheelPowerLvl3 = 0.22;
};

struct C_42mmShooter: C_Config {
  float feedTimeout = 0.5; // Maximum time between consecutive shots, in seconds

  float flywheelPowerLvl0 = 0.15;
  float flywheelPowerLvl1 = 0.17;
  float flywheelPowerLvl2 = 0.2;
  float flywheelPowerLvl3 = 0.22;
};