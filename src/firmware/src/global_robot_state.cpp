#include "global_robot_state.h"

#include "motor_drivers/rm_can_interface.h"

MotorMap::MotorMap(RM_CAN_Interface* rmCAN) {
  this->rmCAN = rmCAN;

  rmCAN->addMotor("FR Drive",   1, CAN1, C620);
  rmCAN->addMotor("FL Drive",   2, CAN1, C620);
  rmCAN->addMotor("BL Drive",   3, CAN1, C620);
  rmCAN->addMotor("BR Drive",   4, CAN1, C620);
  rmCAN->addMotor("Yaw 1",      5, CAN1, C620);
  rmCAN->addMotor("Yaw 2",      6, CAN1, C620);
  rmCAN->addMotor("Pitch L",    1, CAN2, C620);
  rmCAN->addMotor("Pitch R",    2, CAN2, C620);
  rmCAN->addMotor("Flywheel L", 3, CAN2, C620);
  rmCAN->addMotor("Flywheel R", 4, CAN2, C620);
  rmCAN->addMotor("Feeder L",   5, CAN2, C610);
  rmCAN->addMotor("Feeder R",   6, CAN2, C610);

  pid.K[0] = 0.0005;
}

void MotorMap::setMotorRPM(String alias, float rpm, int deltaTime) {
  pid.setpoint = rpm;
  pid.measurement = rmCAN->get_motor_RPM(alias);
  pid.filter(deltaTime);
  rmCAN->set_output(alias, pid.output);
}

void MotorMap::allOff() {
  rmCAN->set_output("FR Drive",   0.0);
  rmCAN->set_output("FL Drive",   0.0);
  rmCAN->set_output("BL Drive",   0.0);
  rmCAN->set_output("BR Drive",   0.0);
  rmCAN->set_output("Yaw 1",      0.0);
  rmCAN->set_output("Yaw 2",      0.0);
  rmCAN->set_output("Pitch L",    0.0);
  rmCAN->set_output("Pitch R",    0.0);
  rmCAN->set_output("Flywheel L", 0.0);
  rmCAN->set_output("Flywheel R", 0.0);
  rmCAN->set_output("Feeder L",   0.0);
  rmCAN->set_output("Feeder R",   0.0);
}