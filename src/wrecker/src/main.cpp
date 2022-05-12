#include <Arduino.h>
#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

#include "state/state.h"
#include "drivers/dr16.h"
#include "state/config.h"
#include "drivers/ref_sys.h"
#include "subsystems/gimbal.h"
#include "drivers/serial_interface.h"
#include "subsystems/swerveChassis.h"




// Loop timing
unsigned long deltaT = 0;
unsigned long lastTime = 0;



// State
S_Robot robot_state;
C_Robot robot_config;


// Subsystems
Gimbal gimbal;
dr16 reciever;
ref_sys refSystem;
SwerveModule sm;
SwerveChassis swerveChassis;

// Runs once
void setup() {
  delay(1000);
  Serial.begin(1000000);
  //Serial.println("basic test");

  // Hardware setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can2.begin();
  can3.begin();

  can1.setBaudRate(1000000);
  can2.setBaudRate(1000000);
  can3.setBaudRate(1000000);

  reciever.init(&robot_state.driverInput);

  // Subsystem setup

  // Subsystem setup
  // gimbal.setup(&robot_config.gimbal, &robot_state);
  // swerveChassis.setup(&robot_config.swerveChassis, &robot_state);

  sm.setup(&robot_config.swerveChassis.FR, &robot_state, &robot_state.chassis.FR);
  dump_Swerve(&robot_state.chassis.FR, "Front_Right");

}


// Runs continuously
void loop() {
  //Serial.println("test");

  dump_Robot(&robot_state, &robot_config);
  
  if (Serial.available() > 0)
    serial_event(&robot_state, &robot_config);

  //reciever.update();
  //gimbal.update(deltaT);
  //swerveChassis.update(deltaT);
  sm.update(0, 0, deltaT);

  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000000) // 1 second
    deltaT = micros() - lastTime;
  
  lastTime = micros();
}


