#ifndef STATE_H
#define STATE_H

struct S_PID {
  float R = 0.0f;
  float Y = 0.0f;
  float X[3] = {0.0f, 0.0f, 0.0f};
};

struct S_SwerveModule {
  int moduleID;

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

struct S_Robot {
  S_Gimbal gimbal;
  S_Chassis chassis;
  S_Shooter Shooter17;
  S_Shooter Shooter42;

  S_RefSystem refSystem;
  DriverInput driverInput;
};

#endif // STATE_H