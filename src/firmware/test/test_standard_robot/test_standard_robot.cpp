#include "global_robot_state.h"
#include "sensors/dr16.h"
#include "gimbal.h"

GlobalRobotState* globalRobotState;
DR16 receiver;
Gimbal gimbal(globalRobotState);

int loopFrequency = 1000; // in microseconds
int programTime; // updates at the end of every loop

void setup() {
  delay(500); // Half-second startup delay

  Serial.begin(1000000);
	Serial.println("-- STANDARD ROBOT TEST PROGRAM --\n");

  globalRobotState->receiver = receiver;

  Serial.print("Waiting for sensor initialization");
  while (!gimbal.ready()) {
    Serial.print(".");
    delay(100);
  }
  Serial.print("Complete!\n");

  programTime = micros();
}

void loop() {
  float deltaTime = micros() - programTime;

  // Read ref, reciever
  globalRobotState->receiver.read();
  globalRobotState->receiver.print_control_data();

  // Generate control output
  //gimbal.loop(deltaTime);

  while (micros() - programTime < loopFrequency) continue;
  programTime = micros();
}