#include "algorithms/pid_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
#include "sensors/revEnc.h"
#include "sensors/refSystem.h"

#ifndef GLOBAL_ROBOT_STATE_H
#define GLOBAL_ROBOT_STATE_H

#define DEMO_CHASSIS_MAX_RPM 8500
#define MATCH_CHASSIS_MAX_RPM 8500
#define DEMO_GIMBAL_YAW_MAX_RPM 2000
#define DEMO_GIMBAL_PITCH_MAX_RPM 2000

enum RobotMode {
  OFF,
  DEMO,
  MATCH
};

enum SystemMode {
  IDLE,
  MANUAL,
  AUTO
};

struct MotorMap {
  MotorMap(RM_CAN_Interface* rmCAN);
  void setMotorRPM(int idx, float rpm, int deltaTime);
  float generateMotorRPMOutput(int idx, float rpm, int deltaTime);
  void allOff();

  RM_CAN_Interface* rmCAN;
  PIDFilter pid;
};

struct GlobalRobotState {
  RM_CAN_Interface rmCAN;

  PIDFilter yawPosPID;
  float desiredYawAngle = 0;
  float yawOffset = 76.72;
  float rawYaw = 0;
  int yawRevs = 0;

  DR16 receiver;
  LSM6DSOX imu;
  RefSystem ref;
  RevEnc xOdometryEncoder = RevEnc(2);
  RevEnc yOdometryEncoder = RevEnc(3);
  RevEnc yawEncoder       = RevEnc(4);
  RevEnc pitchEncoder     = RevEnc(5);
  
  MotorMap motorMap = MotorMap(&rmCAN);
  void setMotorRPM(int idx, float rpm) { motorMap.setMotorRPM(idx, rpm, deltaTime); }
  float generateMotorRPMOutput(int idx, float rpm) { return motorMap.generateMotorRPMOutput(idx, rpm, deltaTime); }

  float deltaTime;

  RobotMode robotMode = OFF;

  SystemMode chassisMode = IDLE;
  float chassisHeading;          // degrees relative to global north
  float chassisHeadingRate;      // rpm

  SystemMode gimbalMode = IDLE;
  float gimbalHeading[2];        // (yaw, pitch): degrees relative to global north
  float gimbalHeadingRate[2];    // (yawRate, pitchRate): rpm

  SystemMode shooterMode = IDLE;
};

#endif