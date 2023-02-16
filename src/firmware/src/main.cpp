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
  if (leftSwitch == 0) {
    state.robotMode == DEMO;
    demoLoop();
  } else if (leftSwitch == 1) {
    state.robotMode == MATCH;
    matchLoop();
  } else {
    state.robotMode == OFF;
    state.motorMap.allOff();
  }

  /* Send CAN output */
  state.rmCAN.write_can();

  while (micros() - programTime < loopFrequency) continue;
}

void demoLoop() {
  if (state.receiver.data[6] == 1) {
    state.setMotorRPM("Flywheel L", -9000.0);
    state.setMotorRPM("Flywheel R", 9000.0);
  }
}

void matchLoop() {

}