#include <FlexCAN_T4.h>
#include "c620.h"

#define G620

CAN_message_t sendMsg;
CAN_message_t recMsg;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
c620CAN controller1 = c620CAN(3, &sendMsg);

#ifdef G6020
short spe = 2000;   //change this to anything between -30000 and 30000 (gm6020) or set in serial
#else
short spe = 2000;   // g620 values range from -16384 to 16384
#endif

//byte bOne = highByte(spe);
//byte bTwo = lowByte(spe);

int del = 500;

//int angle = 0;
//short torque = 0;
//short rpm = 0;

const bool printInfo = true;   //set to true to get stats of motors printed to serial


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);

  Serial.println("Hello from teensy");
}

void loop() {

  controller1.setPower(.1);

  can1.write(sendMsg);

//  Serial.println("Sent CAN data");
//  Serial.println(msg.id,HEX);
//  Serial.print("byte one: ");
//  Serial.println(msg.buf[0], BIN);
//  Serial.print("byte two: ");
//  Serial.println(msg.buf[1], BIN);
//  Serial.println(msg.len);
  
  delay(del);
  if(printInfo && can1.read(recMsg)) {

    controller1.updateMotor(&recMsg);
    
    Serial.println("------------------");
    Serial.print("motor id: ");
    Serial.println(recMsg.id, HEX);
//    angle = recMsg.buf[0];
//    angle = angle << 8;
//    angle = angle | recMsg.buf[1];
    Serial.print("Motor angle: ");
    Serial.println(controller1.getAngle());
//    
//    torque = recMsg.buf[4];
//    torque = torque << 8;
//    torque = torque | recMsg.buf[5];
    Serial.print("Torque: ");
    Serial.println(controller1.getTorque());

//    rpm = recMsg.buf[2];
//    rpm = rpm << 8;
//    rpm = rpm | recMsg.buf[3];
    Serial.print("RPM: ");
    Serial.println(controller1.getRpm());
    
  }

//  if(Serial.available() > 1) {
//    String data = Serial.read();
//    spe = Serial.parseInt();
//    Serial.print("new speed: ");
//    Serial.println(spe);
//    bOne = highByte(spe);
//    bTwo = lowByte(spe);
//  }
  
}
