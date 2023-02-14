//#include "global_robot_state.h"

#include "algorithms/pid_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/revEnc.h"

#include "gimbal.h"

PIDFilter flywheelPID;

RM_CAN_Interface rmCAN;

DR16 receiver;
RevEnc testRevEncoder(2);

//Gimbal gimbal(globalRobotState);

int loopFrequency = 2000; // in microseconds
int programTime; // updates at the end of every loop

void setup() {
  delay(500); // Half-second startup delay

  Serial.begin(1000000);
	Serial.println("-- STANDARD ROBOT TEST PROGRAM --\n");

  //globalRobotState->receiver = receiver;

  // Serial.print("Waiting for sensor initialization");
  // while (!gimbal.ready()) {
  //   Serial.print(".");
  //   delay(100);
  // }
  // Serial.print("Complete!\n");

  flywheelPID.K[0] = 0.0005;

  byte bt[3] = {2,C620,7};
  rmCAN.set_index(0, bt);

  programTime = micros();
}

void loop() {
  int deltaTime = micros() - programTime;
  programTime = micros();

  /* Read ref, reciever */
  // receiver.control_test();
  // if (receiver.read()) receiver.print_control_data();

  // Serial.print(testRevEncoder.getAngle());
  // Serial.print(", ");
  // Serial.println(testRevEncoder.getAngleRaw());

  /* Generate control output */
  //gimbal.loop(deltaTime);

  rmCAN.read_can(2);

  flywheelPID.setpoint = 9000;
  flywheelPID.measurement = rmCAN.get_motor_RPM(0);
  flywheelPID.filter(deltaTime);

  rmCAN.set_output(0, flywheelPID.output);
  rmCAN.write_can();

  while (micros() - programTime < loopFrequency) continue;
}