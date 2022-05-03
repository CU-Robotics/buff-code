#define CONFIG_H

struct C_Teensy {
  int loopStall = 1000; // microseconds
};

struct C_SwerveChassis {
  float drivebaseWidth;
  float drivebaseLength;

  float currentLimitLvl0;
  float currentLimitLvl1;
  float currentLimitLvl2;
  float currentLimitLvl3;

  C_SwerveModule moduleFR;
  C_SwerveModule moduleFL;
  C_SwerveModule moduleBL;
  C_SwerveModule moduleBR;
};

struct C_SwerveModule {
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

struct C_RailChassis {
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

struct C_Gimbal {
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

struct C_17mmShooter {
  float feedRateLow;
  float feedRateHigh;
  float feedRateBurst;

  float flywheelPowerLvl0;
  float flywheelPowerLvl1;
  float flywheelPowerLvl2;
  float flywheelPowerLvl3;
};

struct C_42mmShooter {
  float feedRateLow;
  float feedRateHigh;
  float feedRateBurst;

  float flywheelPowerLvl0;
  float flywheelPowerLvl1;
  float flywheelPowerLvl2;
  float flywheelPowerLvl3;
};