#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "state/state.h"
#include "drivers/dr16.h"
#include "state/config.h"
#include "drivers/dr16.h"
#include "drivers/ref_sys.h"
#include "subsystems/gimbal.h"
#include "drivers/serial_interface.h"
#include "subsystems/swerveChassis.h"

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
CAN_message_t canRecieveMessages[3][11];
CAN_message_t tempMessage;

// Loop timing
unsigned long deltaT = 5000;
unsigned long lastTime = 0;

// State
S_Robot robot_state;
C_Robot robot_config;

// Subsystems
// Gimbal gimbal;
// dr16 reciever;
// ref_sys refSystem;
// SwerveModule sm;
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


  // Subsystem setup

  // Subsystem setup
  // reciever.init(&robot_state.driverInput);
  // gimbal.setup(&robot_config.gimbal, &robot_state);
  swerveChassis.setup(&robot_config.swerveChassis, &robot_state);

  dump_Robot(&robot_config, &robot_state);
}


// Runs continuously
void loop() {

  // Necesary for motors to recieve data over CAN
  // Needs to have usage
  //  can.update(XXX, YYY);
  //
  // while (can1.read(tempMessage))
  //   canRecieveMessages[0][tempMessage.id - 0x201] = tempMessage;
  //
  // while (can2.read(tempMessage))
  //   canRecieveMessages[1][tempMessage.id - 0x201] = tempMessage;
  //
  // while (can3.read(tempMessage))
  //   canRecieveMessages[2][tempMessage.id - 0x201] = tempMessage;
  
  dump_Robot(&robot_config, &robot_state);
  
  if (Serial.available() > 0)
    serial_event(&robot_config, &robot_state);

  //reciever.update();
  //gimbal.update(deltaT);
  swerveChassis.update(deltaT);

  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;

  while (deltaT < 1000000) // 1 second
    deltaT = micros() - lastTime;
  
  lastTime = micros();
}


