#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "c620.h"

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

c620 myMotor(1, &msg);
// c620 myMotor();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);
  Serial.println("test");
  Serial.println(msg.id);
  delay(1000);
  //myMotor.setPower
}

void loop() {
  Serial.print("id: ");
  Serial.println(msg.id);
  delay(500);
}
