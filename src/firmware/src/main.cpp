//#include "global_robot_state.h"

#include "algorithms/pid_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
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
  // for (float value : state.rmCAN.motor_arr[8].data) {
  //   Serial.printf("%f, ", value);
  // }
  // Serial.println();
  float ratio = 17.0 / 246.0;
  Serial.printf("rpm: %f, %f, %f\n", state.rmCAN.get_motor_RPM(8), state.rmCAN.get_motor_RPM(4), state.imu.data[5] / 6.0);

  float rpm = state.imu.data[5] / 6.0 / ratio;
  float rpm2 = -state.rmCAN.get_motor_RPM(8);
  //state.setMotorRPM(8, rpm);//state.receiver.out.l_stick_x * 0);
  //state.setMotorRPM(4, rpm);//state.receiver.out.l_stick_x * 0);

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
}

void matchLoop() {

}

void loop() {
  uint32_t currentTime = micros();
  state.deltaTime = (currentTime - programTime) / 1000.0;
  programTime = currentTime;

  /* Read sensors */
  state.receiver.read();
  state.imu.read_lsm6dsox_gyro();
  // for (float value : state.imu.data) {
  //   Serial.printf("%f, ", value);
  // }
  //Serial.println(state.imu.data[5] / 6.0);

  state.rmCAN.read_can(0);
  state.rmCAN.read_can(CAN1);
  state.rmCAN.read_can(CAN2);

  /* Generate control output */
  if (state.receiver.out.l_switch == 3) {
    state.robotMode = DEMO;
    demoLoop();
  } else if (state.receiver.out.l_switch == 2) {
    state.robotMode = MATCH;
    matchLoop();
    state.rmCAN.set_output(8, 0);
    state.rmCAN.set_output(4, 0);
  } else {
    state.robotMode = OFF;
    state.rmCAN.set_output(8, 0);
    state.rmCAN.set_output(4, 0);
    //state.motorMap.allOff();
  }

  /* Send CAN output */
  state.rmCAN.write_can();

  // Serial.println(state.deltaTime);

  while (micros() - programTime < loopFrequency) continue;
}