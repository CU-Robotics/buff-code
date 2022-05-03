#define STATE_H

struct S_SwerveChassis {
  float heading;
  float rpm;
  float alpha; // angular acceleration
  float xAccel;
  float yAccel;
};

struct S_RailChassis {
  float pos;
  float vel;
  float accel;
};

struct S_Gimbal {
  float pitch;
  float yaw;
  float yawGlobal;
};

struct S_17mmShooter {
  bool firing;
};

struct S_42mmShooter {
  bool firing;
};

struct S_DriverInput {
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
};

struct S_RefSystem {
  short robotLevel;
  int matchTime;
};