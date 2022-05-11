#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "state/state.h"
#include "state/config.h"
#include "subsystems/swerveChassis.h"


// Loop timing
unsigned long deltaT = 0;
unsigned long lastTime = 0;

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// State
S_Robot robot_state;
C_SwerveChassis swerve_config;

// Subsystems
SwerveChassis swerve_Chassis;

// Runs once
void setup() {
  // Hardware setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can2.begin();
  can3.begin();

  can1.setBaudRate(1000000);
  can2.setBaudRate(1000000);
  can3.setBaudRate(1000000);

  // Subsystem setup
  swerve_Chassis.setup(&swerve_config, &robot_state);
}


// Runs continuously
void loop() {
  //swerveChassisSubsystem.update(deltaT);
  dump_Robot_State(&robot_state);
  delay(0.1);
  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000) {
    deltaT = micros() - lastTime;
  }
  lastTime = micros();
}


