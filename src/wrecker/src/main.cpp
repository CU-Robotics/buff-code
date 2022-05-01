#include <Arduino.h>

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

unsigned long deltaT = 0;
unsigned long lastTime = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chassisCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> superStructureCAN;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  chassisCAN.begin();
  superStructureCAN.begin();
  chassisCAN.setBaudRate(1000000);
  superStructureCAN.setBaudRate(1000000);
}

void loop() {


  // delta-time calculator: keep this at the bottom
  deltaT = micros() - lastTime;
  while (deltaT < 1000) {
    deltaT = micros() - lastTime;
  }
  lastTime = micros();
}