#ifndef STATE_H
#define STATE_H
#include <stdint.h>

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

  S_PID drivePos;
  S_PID driveVel;
};


struct S_Gimbal {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yawGlobal = 0.0f;
  float yaw_reference = 0.0f;
  float pitch_reference = 0.0f;

  S_PID yaw_PID;
  S_PID pitch_PID;
};

struct S_Shooter {
  bool firing = false;

  S_PID feedPID;
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

  int16_t mouseX = 0;
  int16_t mouseY = 0;
  int16_t mouseZ = 0;
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
    char curr_stage = '\r';
    char comp_type = '\r';
    char comp_result = '\r';

    int rem_time = -1;

    int red_hero_hp = -1;
    int red_sentry_hp = -1;
    int red_infantry_hp = -1;
    int blue_hero_hp = -1;
    int blue_sentry_hp = -1;
    int blue_infantry_hp = -1;
    int red_hero_max_hp = -1;
    int red_sentry_max_hp = -1;
    int red_infantry_max_hp = -1;
    int blue_hero_max_hp = -1;
    int blue_sentry_max_hp = -1;
    int blue_infantry_max_hp = -1;

    int red_one_rem_proj = -1;
    int red_two_rem_proj = -1;
    int blue_one_rem_proj = -1;
    int blue_two_rem_proj = -1;

    char ref_warning = '\r';
    int foul_robot_id = -1;
    
    int red_hero_robot_level = -1;
    int red_infantry_robot_level = -1;
    int red_sentry_robot_level = -1;
    int blue_hero_robot_level = -1;
    int blue_infantry_robot_level = -1;
    int blue_sentry_robot_level = -1;

    int robot_1_cool_val = -1;       //17mm
    int robot_1_barr_heat_lim = -1;       //17mm
    int robot_1_speed_lim = -1;       //17mm

    int robot_2_cool_val = -1;       //17mm
    int robot_2_barr_heat_lim = -1;       //17mm
    int robot_2_speed_lim = -1;       //17mm

    int robot_42_cool_val = -1;
    int robot_42_heat_lim = -1;
    int robot_42_speed_lim = -1;  

    int robot_power_lim = -1;

    int chasis_volt = -1;
    int chasis_current = -1;

    int robot_buff = -1;

    int launch_freq = -1;
    int launch_speed = -1;

    int rem_17_proj = -1;
    int rem_42_proj = -1;
};

struct S_Robot {
  S_Gimbal gimbal;
  S_Chassis chassis;
  S_Shooter shooter17;
  S_Shooter shooter42;

  S_RefSystem refSystem;
  DriverInput driverInput;
};

#endif // STATE_H