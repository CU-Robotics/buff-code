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
  /* Chassis */

  float x = state.receiver.out.l_stick_x;
  float y = state.receiver.out.l_stick_y;
  float spin = state.receiver.out.r_stick_x;

  int max_rpm = 8000;

  double speed_fr = y - x - spin;
  double speed_fl = -(y + x + spin);
  double speed_bl = -(y - x + spin);
  double speed_br = y + x - spin;

  if (isnan(speed_fr)) speed_fr = 0;
  if (isnan(speed_fl)) speed_fl = 0;
  if (isnan(speed_bl)) speed_bl = 0;
  if (isnan(speed_br)) speed_br = 0;

  double max_speed = fabs(speed_fr);
  max_speed = max(max_speed, fabs(speed_fl));
  max_speed = max(max_speed, fabs(speed_bl));
  max_speed = max(max_speed, fabs(speed_br));

  if (max_speed > 1.0) {
    speed_fr /= max_speed;
    speed_fl /= max_speed;
    speed_bl /= max_speed;
    speed_br /= max_speed;
  }

  state.setMotorRPM(3, speed_fr * max_rpm);
  state.setMotorRPM(1, speed_fl * max_rpm);
  state.setMotorRPM(6, speed_bl * max_rpm);
  state.setMotorRPM(5, speed_br * max_rpm);

  /* Gimbal */
  if (state.receiver.out.l_stick_y != -1.0) {
    // Manual control
    float ratio = 17.0 / 246.0;
    float rpm = state.imu.data[5] / 6.0 / ratio;
    state.setMotorRPM(8, rpm);
    state.setMotorRPM(4, rpm);
  } else {
    // Autoaim
  }

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

  state.rmCAN.read_can(CAN1);
  state.rmCAN.read_can(CAN2);

  /* Generate control output */
  if (state.receiver.out.l_switch == 3) {
    state.robotMode = DEMO;
    demoLoop();
  } else if (state.receiver.out.l_switch == 2) {
    state.robotMode = MATCH;
    matchLoop();
  } else {
    state.robotMode = OFF;
    state.motorMap.allOff();
  }

  /* Send CAN output */
  state.rmCAN.write_can();

  //Serial.println(state.deltaTime);

  while (micros() - programTime < loopFrequency) continue;
}