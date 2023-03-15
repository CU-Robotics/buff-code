#include "global_robot_state.h"

#include "motor_drivers/rm_can_interface.h"

MotorMap::MotorMap(RM_CAN_Interface* rmCAN) {
  this->rmCAN = rmCAN;

  byte x_fr[3] = {CANBUS_1, C620, 1};
  rmCAN->set_index(1, x_fr);
  byte x_fl[3] = {CANBUS_1, C620, 2};
  rmCAN->set_index(2, x_fl);
  byte x_bl[3] = {CANBUS_1, C620, 3};
  rmCAN->set_index(3, x_bl);
  byte x_br[3] = {CANBUS_1, C620, 7};
  rmCAN->set_index(7, x_br);

  byte yaw_1[3] = {CANBUS_1, C610, 5};
  rmCAN->set_index(5, yaw_1);
  byte yaw_2[3] = {CANBUS_1, C620, 6};
  rmCAN->set_index(6, yaw_2);

  byte pitch_l[3] = {CANBUS_2, C620, 1};
  rmCAN->set_index(9, pitch_l);
  byte pitch_r[3] = {CANBUS_2, C620, 2};
  rmCAN->set_index(10, pitch_r);

  byte flywheel_l[3] = {CANBUS_2, C620, 3};
  rmCAN->set_index(11, flywheel_l);
  byte flywheel_r[3] = {CANBUS_2, C620, 4};
  rmCAN->set_index(12, flywheel_r);

  byte feeder_l[3] = {CANBUS_2, C610, 5};
  rmCAN->set_index(13, feeder_l);
  byte feeder_r[3] = {CANBUS_2, C610, 6};
  rmCAN->set_index(14, feeder_r);

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

  pid.K[0] = 0.00045;
}

void MotorMap::setMotorRPM(int idx, float rpm, int deltaTime) {
  pid.setpoint = rpm;
  pid.measurement = rmCAN->get_motor_RPM(idx);
  pid.filter(deltaTime);
  rmCAN->set_output(idx, constrain(pid.output, -1.0, 1.0));
}

float MotorMap::generateMotorRPMOutput(int idx, float rpm, int deltaTime) {
  pid.setpoint = rpm;
  pid.measurement = rmCAN->get_motor_RPM(idx);
  pid.filter(deltaTime);
  return constrain(pid.output, -1.0, 1.0);
}

void MotorMap::allOff() {
  for (int i = 0; i < 16; i++)
    rmCAN->set_output(i, 0);
}