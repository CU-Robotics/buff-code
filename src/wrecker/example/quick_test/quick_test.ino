#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

short spe = 2000;   //change this to anything between -30000 and 30000 (gm6020) or set in serial

byte bOne = highByte(16384);
byte bTwo = lowByte(16384);



int del = 5;

int angle = 0;
short torque = 0;
short rpm = 0;

const bool printInfo = false;   //set to true to get stats of motors printed to serial

CAN_message_t msg;
CAN_message_t recMsg;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);

}

void loop() {
  msg.id = 0x200;    //Modify id if not using gm6020
  
  msg.buf[0] = highByte(16384); //set high order byte
  msg.buf[1] = lowByte(16384); //set low order byte
  
  msg.buf[2] = highByte(16384); //set high order byte
  msg.buf[3] = lowByte(16384); //set low order byte

  msg.buf[4] = highByte(16384); //set high order byte
  msg.buf[5] = lowByte(16384); //set low order byte

  msg.buf[6] = bOne; //set high order byte
  msg.buf[7] = bTwo; //set low order byte

  can1.write(msg);



//  Serial.println("Sent CAN data");
//  Serial.println(msg.id,HEX);
//  Serial.print("byte one: ");
//  Serial.println(msg.buf[0], BIN);
//  Serial.print("byte two: ");
//  Serial.println(msg.buf[1], BIN);
//  Serial.println(msg.len);
  
  delay(del);
  if(printInfo && can1.read(recMsg)) {
    Serial.println("------------------");
    Serial.print("motor id: ");
    Serial.println(recMsg.id, HEX);
    angle = recMsg.buf[0];
    angle = angle << 8;
    angle = angle | recMsg.buf[1];
    Serial.print("Motor angle: ");
    Serial.println(angle);
    
    torque = recMsg.buf[4];
    torque = torque << 8;
    torque = torque | recMsg.buf[5];
    Serial.print("Torque: ");
    Serial.println(torque);

    rpm = recMsg.buf[2];
    rpm = rpm << 8;
    rpm = rpm | recMsg.buf[3];
    Serial.print("RPM: ");
    Serial.println(rpm);
    
  }

  if(Serial.available() > 1) {
//    String data = Serial.read();
    spe = Serial.parseInt();
    Serial.print("new speed: ");
    Serial.println(spe);
    bOne = highByte(spe);
    bTwo = lowByte(spe);
  }
  
}
