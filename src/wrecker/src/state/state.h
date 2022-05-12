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

void dump_DriverInput(DriverInput*, char*);

struct S_RefSystem {
    char curr_stage;
    char comp_type;
    int rem_time;
    char comp_result;

    int red_hero_hp;
    int red_sentry_hp;
    int red_infantry_hp;
    int blue_hero_hp;
    int blue_sentry_hp;
    int blue_infantry_hp;
    int red_hero_max_hp;
    int red_sentry_max_hp;
    int red_infantry_max_hp;
    int blue_hero_max_hp;
    int blue_sentry_max_hp;
    int blue_infantry_max_hp;

    int red_one_rem_proj;
    int red_two_rem_proj;
    int blue_one_rem_proj;
    int blue_two_rem_proj;

    char ref_warning;
    int foul_robot_id;
    
    int red_hero_robot_level;
    int red_infantry_robot_level;
    int red_sentry_robot_level;
    int blue_hero_robot_level;
    int blue_infantry_robot_level;
    int blue_sentry_robot_level;

    int robot_1_cool_val;       //17mm
    int robot_1_barr_heat_lim;       //17mm
    int robot_1_speed_lim;       //17mm

    int robot_2_cool_val;       //17mm
    int robot_2_barr_heat_lim;       //17mm
    int robot_2_speed_lim;       //17mm

    int robot_42_cool_val;
    int robot_42_heat_lim;
    int robot_42_speed_lim;  

    int robot_power_lim;

    int chasis_volt;
    int chasis_current;

    int robot_buff;

    int launch_freq;
    int launch_speed;

    int rem_17_proj;
    int rem_42_proj;
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