#include <Arduino.h>

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "state/config.h"
#include "state/state.h"

#include "subsystems/swerveChassis.h"


// Loop timing
unsigned long deltaT = 0;
unsigned long lastTime = 0;

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chassisCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> superStructureCAN;

// State
S_Robot s_robot;
C_SwerveChassis c_swerveChassis;

// Subsystems
SwerveChassis swerveChassisSubsystem;

// Runs once
void setup() {
  // Hardware setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  chassisCAN.begin();
  superStructureCAN.begin();
  chassisCAN.setBaudRate(1000000);
  superStructureCAN.setBaudRate(1000000);

  // Subsystem setup
  swerveChassisSubsystem.setup(&c_swerveChassis, &s_robot);
}


// Runs continuously
void loop() {
  swerveChassisSubsystem.loop(deltaT);

  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000) {
    deltaT = micros() - lastTime;
  }
  lastTime = micros();
}