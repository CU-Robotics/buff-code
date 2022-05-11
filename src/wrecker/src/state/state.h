#ifndef STATE_H
#define STATE_H

struct S_PID {
  float R = 0.0f;
  float Y = 0.0f;
  float X[3] = {0.0f, 0.0f, 0.0f};
};

void dump_PID_State(S_PID*, char*);

struct S_SwerveModule {
  float steer_angle;
  float steer_speed;
  float drive_speed;
  float drive_accel;
};

void dump_Swerve_State(S_SwerveModule*, char*);

struct S_Chassis {
  float heading; // naming needs to be more consistent (greek letters, descriptive or units)
  float rpm;
  float alpha; // angular acceleration
  float a[2];
  S_SwerveModule fl;
  S_SwerveModule fr;
  S_SwerveModule rr;
  S_SwerveModule rl;

};

void dump_Chassis_State(S_Chassis*, char*);

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
  S_Chassis chassis;
  S_Gimbal gimbal;
  S_Shooter Shooter17;
  S_Shooter Shooter42;

  DriverInput driverInput;
  S_RefSystem refSystem;
};

void dump_Robot_State(S_Robot*);

#endif // STATE_H