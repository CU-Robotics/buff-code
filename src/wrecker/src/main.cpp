#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

#include "state/state.h"
#include "state/config.h"
#include "drivers/serial_interface.h"
#include "subsystems/swerveChassis.h"


// Loop timing
unsigned long deltaT = 0;
unsigned long lastTime = 0;

// Robot Objects
S_Robot robot_state;
C_Robot robot_config;


// Subsystems
SwerveChassis swerve_Chassis;

// Runs once
void setup() {
  // Hardware setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Subsystem setup
  swerve_Chassis.setup(&robot_config.swerveChassis, &robot_state);

  Serial.begin(1000000);
}


// Runs continuously
void loop() {

  dump_Robot(&robot_state, &robot_config);
  
  if (Serial.available() > 0)
    serial_event(&robot_state, &robot_config);

  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000) {
    deltaT = micros() - lastTime;
  }
  lastTime = micros();
  delay(100);
}


