#include "global_robot_state.h"

#include "motor_drivers/rm_can_interface.h"

MotorMap::MotorMap(RM_CAN_Interface* rmCAN) {
  this->rmCAN = rmCAN;

  byte b6[3] = {1, 1, 6};
  rmCAN->set_index(6, b6);

  byte b5[3] = {1, 1, 5};
  rmCAN->set_index(5, b5);

  byte b3[3] = {1, 1, 3};
  rmCAN->set_index(3, b3);

  byte b1[3] = {1, 1, 1};
  rmCAN->set_index(1, b1);

  byte b4[3] = {1, 1, 4};
  rmCAN->set_index(4, b4);

  byte b8[3] = {1, 1, 8};
  rmCAN->set_index(8, b8);


  //rmCAN->addMotor("Yaw 1", 8, CAN1, C620);

  // rmCAN->addMotor("FR Drive",   1, CAN1, C620);
  // rmCAN->addMotor("FL Drive",   2, CAN1, C620);
  // rmCAN->addMotor("BL Drive",   3, CAN1, C620);
  // rmCAN->addMotor("BR Drive",   4, CAN1, C620);
  // rmCAN->addMotor("Yaw 1",      5, CAN1, C620);
  // rmCAN->addMotor("Yaw 2",      6, CAN1, C620);
  // rmCAN->addMotor("Pitch L",    1, CAN2, C620);
  // rmCAN->addMotor("Pitch R",    2, CAN2, C620);
  // rmCAN->addMotor("Flywheel L", 3, CAN2, C620);
  // rmCAN->addMotor("Flywheel R", 4, CAN2, C620);
  // rmCAN->addMotor("Feeder L",   5, CAN2, C610);
  // rmCAN->addMotor("Feeder R",   6, CAN2, C610);

  pid.K[0] = 0.0002;
  //pid.K[2] = 0.0004;
}

void MotorMap::setMotorRPM(int idx, float rpm, int deltaTime) {
  pid.setpoint = rpm;
  pid.measurement = rmCAN->get_motor_RPM(idx);
  pid.filter(deltaTime);
  rmCAN->set_output(idx, pid.output);
}

void MotorMap::setMotorRPM(String alias, float rpm, int deltaTime) {
  pid.setpoint = rpm;
  pid.measurement = rmCAN->get_motor_RPM(alias);
  pid.filter(deltaTime);
  rmCAN->set_output(alias, pid.output);
}

void MotorMap::allOff() {
  for (int i = 0; i < sizeof(rmCAN->motor_arr); i++)
    rmCAN->set_output(i, 0.0);
}