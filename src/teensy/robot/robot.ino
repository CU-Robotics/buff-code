<<<<<<< HEAD
//#include <FlexCAN_T4.h>
#include "./libraries/FlexCAN_T4-master/FlexCAN_T4.h"
=======
#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif
>>>>>>> 271e6de3c568cb81be56fc9e807b85d9e091d922

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
  myMotor.setPower(.5);
}

void loop() {
  Serial.print("id: ");
  Serial.println(msg.id, HEX);
  Serial.print("power: ");
  short temp = msg.buf[0];
  temp = temp << 8;
  temp = temp | msg.buf[1];
  Serial.println(temp);
  Serial.println(myMotor.getTemp());
  delay(1000);
}
