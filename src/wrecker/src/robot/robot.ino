#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "c620.h"
#include "gimbal.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chassisCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> SuperStructureCAN;

CAN_message_t chassisSendMsg;
CAN_message_t chassisRecMsg;

CAN_message_t superStructureSendMsg;
CAN_message_t superStructureRecMsg;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  chassisCAN.begin();
  SuperStructureCAN.begin();
  chassisCAN.setBaudRate(1000000);
  SuperStructureCAN.setBaudRate(1000000);
  Serial.begin(9600);
}

void loop() {
  gimbal.update();
  chassis.update();
  shooter.update();

  delay(10);
}
