#ifndef STATE_H
#define STATE_H

struct S_PID {
  float R = 0.0f;
  float Y = 0.0f;
  float X[3] = {0.0f, 0.0f, 0.0f};
};

struct S_SwerveModule {
  int moduleID;
  int steerMotorID;
  int steerEncoderID;
  int driveMotorID;

  float steer_angle;
  float steer_speed;
  float drive_speed;
  float drive_accel;

  S_PID steerVel;
  S_PID steerPos;
  S_PID driveVel;
};

struct S_Chassis {
  float heading; // naming needs to be more consistent (greek letters, descriptive or units)
  float rpm;
  float alpha; // angular acceleration
  float a[2];
  
  S_SwerveModule FL;
  S_SwerveModule FR;
  S_SwerveModule RR;
  S_SwerveModule RL;
};

struct S_Gimbal {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yawGlobal = 0.0f;
  S_PID yaw_PID;
  S_PID pitch_PID;
};

struct S_Shooter {
  bool firing = false;
};

struct DriverInput {
  float leftStickX = 0;
  float leftStickY = 0;
  float rightStickX = 0;
  float rightStickY = 0;
  short leftSwitch = 0;
  short rightSwitch = 0;
  uint16_t remoteWheel;

  uint8_t s1;
  uint8_t s2;

  float mouseX;
  float mouseY;
  float mouseZ;
  bool mouseLeft;
  bool mouseRight;

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
  bool v;
  bool b;
  bool shift;
  bool ctrl;
  //byte keyboard[2] = 15 bit value
};

struct S_RefSystem {
  short robotLevel;
  int matchTime;
};

struct S_Robot {
  S_Gimbal gimbal;
  S_Chassis chassis;
  S_Shooter Shooter17;
  S_Shooter Shooter42;

  S_RefSystem refSystem;
  DriverInput driverInput;
};

#endif // STATE_H