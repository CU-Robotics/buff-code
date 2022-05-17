#ifndef STATE_H
#define STATE_H

struct S_PID {
  float R = 0.0f;
  float Y = 0.0f;
  float X[3] = {0.0f, 0.0f, 0.0f};
};

struct S_SwerveModule {
  float steer_angle = 0.0f;
  float steer_speed = 0.0f;
  float drive_speed = 0.0f;
  float drive_accel = 0.0f;

  S_PID steerVel;
  S_PID steerPos;
  S_PID driveVel;
};

struct S_Chassis {
  float rpm = 0.0f;
  float alpha = 0.0f; // angular acceleration
  float heading = 0.0f; // naming needs to be more consistent (greek letters, descriptive or units)
  float a[2] = {0.0f, 0.0f};

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
  float leftStickX = 0.0f;
  float leftStickY = 0.0f;
  float rightStickX = 0.0f;
  float rightStickY = 0.0f;
  short leftSwitch = 0.0f;
  short rightSwitch = 0.0f;
  uint16_t remoteWheel = 0;

  uint8_t s1 = 0;
  uint8_t s2 = 0;

  float mouseX = 0.0f;
  float mouseY = 0.0f;
  float mouseZ = 0.0f;
  bool mouseLeft = false;
  bool mouseRight = false;

  bool w = false;
  bool a = false;
  bool s = false;
  bool d = false;
  bool q = false;
  bool e = false;
  bool r = false;
  bool f = false;
  bool g = false;
  bool z = false;
  bool x = false;
  bool c = false;
  bool v = false;
  bool b = false;
  bool shift = false;
  bool ctrl = false;

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