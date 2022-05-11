#ifndef STATE_H
#define STATE_H

struct S_PID {
  float R = 0.0f;
  float Y = 0.0f;
  float X[3] = {0.0f, 0.0f, 0.0f};
};

void dump_PID_State(S_PID*, char*);

struct S_SwerveChassis {
  float heading;
  float rpm;
  float alpha; // angular acceleration
  float xAccel;
  float yAccel;
};

void dump_SwerveChassis_State(S_SwerveChassis*, char*);

struct S_RailChassis {
  float pos;
  float vel;
  float accel;
};

void dump_RailChassis_State(S_RailChassis*, char*);

struct S_Gimbal {
  float yaw;
  float pitch;
  float yawGlobal;
  S_PID yaw_PID;
  S_PID pitch_PID;
};

void dump_Gimbal_State(S_Gimbal*, char*);

struct S_Shooter {
  bool firing;
};

void dump_Shooter_State(S_Shooter*, char*);

struct DriverInput {
  float leftStickX;
  float leftStickY;
  float rightStickX;
  float rightStickY;
  short leftSwitch;
  short rightSwitch;

  float mouseX;
  float mouseY;

  bool w;
  bool a;
  bool s;
  bool d;
  bool q;
  bool e;
  bool r;
  bool f;
  bool g;
  bool z;
  bool x;
  bool c;
  bool space;
  bool shift;
  bool ctrl;
  //byte keyboard[2] = 15 bit value
};

void dump_DriverInput(DriverInput*, char*);

struct S_RefSystem {
  short robotLevel;
  int matchTime;
};

void dump_RefSystem_State(S_RefSystem*, char*);

struct S_Robot {
  S_SwerveChassis swerve_chassis;
  S_RailChassis rail_chassis;
  S_Gimbal gimbal;
  S_Shooter Shooter17;
  S_Shooter Shooter42;

  DriverInput driverInput;
  S_RefSystem refSystem;
};

void dump_Robot_State(S_Robot*);

#endif // STATE_H