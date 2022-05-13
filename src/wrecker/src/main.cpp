#include <Arduino.h>
#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
CAN_message_t canRecieveMessages[3][11];
CAN_message_t tempMessage;

#include "state/state.h"
#include "state/config.h"
#include "subsystems/swerveChassis.h"
#include "drivers/dr16.h"

#include "drivers/ref_sys.h"


// Loop timing
unsigned long deltaT = 5000;
unsigned long lastTime = 0;



// State
S_Robot robot_state;
C_SwerveChassis swerve_config;

// Subsystems
SwerveChassis swerveChassis;
dr16 reciever;
ref_sys refSystem;

// Runs once
void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("basic test");

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
  swerveChassis.setup(&swerve_config, &robot_state);
}


// Runs continuously
void loop() {
  //Necesary for motors to recieve data over CAN
  while (can1.read(tempMessage))
  {
    // Serial.print("got message on can1 from ");
    // Serial.println(tempMessage.id, HEX);
    canRecieveMessages[0][tempMessage.id - 0x201] = tempMessage;
  }
  
  while (can2.read(tempMessage))
  {
    // Serial.print("got message on can2 from ");
    // Serial.println(tempMessage.id, HEX);
    canRecieveMessages[1][tempMessage.id - 0x201] = tempMessage;
  }

  while (can3.read(tempMessage))
  {
    // Serial.print("got message on can3 from ");
    // Serial.println(tempMessage.id, HEX);
    canRecieveMessages[2][tempMessage.id - 0x201] = tempMessage;
  }
  

  // Serial.println("test");

  reciever.update();

  // Serial.println("Finished reciever update");

  swerveChassis.update(deltaT);

  // Serial.println("finsished swervechassis update");

  // Delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000) {
    deltaT = micros() - lastTime;
  }
  lastTime = micros();
}


