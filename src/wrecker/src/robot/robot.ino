#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "c620.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;
CAN_message_t recMsg;

short angle;
short torque;
short rpm;


c620CAN myMotor(1, &msg);
// c620 myMotor();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);
  // myMotor.setPower(0.5);
}

void loop() {
  // Serial.print("id: ");
  // Serial.println(msg.id, HEX);
  myMotor.setPower(1);
  Serial.print("send id: ");
  Serial.println(msg.id, HEX);
  Serial.println("send power: ");
  short temp = msg.buf[4];
  temp = temp << 8;
  temp = temp | msg.buf[5];
  Serial.println(temp);
  // Serial.println(myMotor.getTemp());
  can1.write(msg);

  // if(can1.read(recMsg)) {   //parsed without motor class
  //   Serial.println("------------------");
  //   Serial.print("motor id: ");
  //   Serial.println(recMsg.id, HEX);
  //   angle = recMsg.buf[0];
  //   angle = angle << 8;
  //   angle = angle | recMsg.buf[1];
  //   Serial.print("Motor angle: ");
  //   Serial.println(angle);
    
  //   torque = recMsg.buf[4];
  //   torque = torque << 8;
  //   torque = torque | recMsg.buf[5];
  //   Serial.print("Torque: ");
  //   Serial.println(torque);

  //   rpm = recMsg.buf[2];
  //   rpm = rpm << 8;
  //   rpm = rpm | recMsg.buf[3];
  //   Serial.print("RPM: ");
  //   Serial.println(rpm);
  // }

  if(can1.read(recMsg)) { //parsed with motor class
    myMotor.updateMotor(recMsg);
    Serial.print("angle: ")
    Serial.println(myMotor.getAngle());
    Serial.print("rpm: ");
    Serial.println(myMotor.getRPM());
    Serial.print("torque: ");
    Serial.println(myMotor.getTorque());
    Serial.print("temp: ")
    Serial.println(myMotor.getTemp());
  }

  delay(10);
}
