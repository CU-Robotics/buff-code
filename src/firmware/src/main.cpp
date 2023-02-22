//#include "global_robot_state.h"

#include "algorithms/pid_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/revEnc.h"

#include "global_robot_state.h"

GlobalRobotState state;

uint32_t loopFrequency = 2000; // in microseconds
uint32_t programTime; // stores the system time at the start of every loop

void setup() {
  delay(500); // Half-second startup delay

  Serial.begin(1000000);
	Serial.println("-- STANDARD ROBOT TEST PROGRAM --\n");

  // Serial.print("Waiting for sensor initialization");
  // while ( {Sensors are outputting garbage data} ) {
  //   Serial.print(".");
  //   delay(100);
  // }
  // Serial.print("Complete!\n");

  programTime = micros();
}

void demoLoop() {
  // Gimbal
  // if (state.receiver.data[1] != -1.0) {
  //   // Manual control
  //   float yawRate = state.receiver.data[0] * 3000.0; // Multiply by max RPM allowed in demo mode
  //   float maxPitchRPM = 500.0;
  //   if (state.pitchEncoder.getAngle()) {
  //     state.pitchEncoder.getAngle() * 0.01;
  //   }
  //   state.setMotorRPM("Yaw 1", yawRate);
  //   state.setMotorRPM("Yaw 2", yawRate);

  //   float pitchRate = state.receiver.data[1] * 500.0; // Multiply by max RPM allowed in demo mode
  //   float pitchGravityFeedForward = cosf(1.0/* Pitch Angle * pi / 180 */);
  //   state.setMotorRPM("Pitch L", -pitchRate);
  //   state.setMotorRPM("Pitch R", pitchRate);
  // } else {
  //   // Autoaim
  // }

  // // Shooter
  // if (state.receiver.data[6] == 1) {
  //   state.setMotorRPM("Flywheel L", -9000.0);
  //   state.setMotorRPM("Flywheel R", 9000.0);
  // }

  //Serial.println(state.receiver.data[0]);

  for (int i = 0; i < sizeof(state.receiver.data); i++) Serial.print(state.receiver.data[i]);
	Serial.println();
}

void matchLoop() {

}

void loop() {
  uint32_t currentTime = micros();
  state.deltaTime = currentTime - programTime;
  programTime = currentTime;

  /* Read sensors */
  state.receiver.read();
  state.rmCAN.read_can(1);
  state.rmCAN.read_can(2);

  /* Generate control output */
  int leftSwitch = (int)(state.receiver.data[5]);
  if (leftSwitch == 3) {
    state.robotMode = DEMO;
    demoLoop();
  } else if (leftSwitch == ASCII_21) {
    state.robotMode = MATCH;
    matchLoop();
  } else {
    state.robotMode = OFF;
    state.motorMap.allOff();
  }
  demoLoop();

  /* Send CAN output */
  state.rmCAN.write_can();

  while (micros() - programTime < loopFrequency) continue;
}