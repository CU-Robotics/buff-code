#include "algorithms/pid_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/revEnc.h"

#ifndef GLOBAL_ROBOT_STATE_H
#define GLOBAL_ROBOT_STATE_H

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

struct GlobalRobotState {
  RM_CAN_Interface rmCAN;

  DR16 receiver;
  RevEnc yawEncoder       = RevEnc(1);
  RevEnc pitchEncoder     = RevEnc(2);
  RevEnc xOdometryEncoder = RevEnc(3);
  RevEnc yOdometryEncoder = RevEnc(4);

  MotorMap motorMap = MotorMap(&rmCAN);
  void setMotorRPM(String alias, float rpm) { motorMap.setMotorRPM(alias, rpm, deltaTime); }

  int deltaTime;

  RobotMode robotMode = OFF;

  SystemMode chassisMode = IDLE;
  float chassisHeading;          // degrees relative to global north
  float chassisHeadingRate;      // rpm

  SystemMode gimbalMode = IDLE;
  float gimbalHeading[2];        // (yaw, pitch): degrees relative to global north
  float gimbalHeadingRate[2];    // (yawRate, pitchRate): rpm

  SystemMode shooterMode = IDLE;
};

struct MotorMap {
  MotorMap(RM_CAN_Interface* rmCAN);
  void setMotorRPM(String alias, float rpm, int deltaTime);
  void allOff();

  RM_CAN_Interface* rmCAN;
  PIDFilter pid;
};

#endif