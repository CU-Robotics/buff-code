#include <Arduino.h>

#include "state/state.h"
#include "drivers/c620.h"

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

extern CAN_message_t canRecieveMessages[3][11];

CAN_message_t c6x0Messages[3][2];

void sendC6x0() {
  can1.write(c6x0Messages[0][0]);
  can1.write(c6x0Messages[0][1]);
  can2.write(c6x0Messages[1][0]);
  can2.write(c6x0Messages[1][1]);
  // can3.write(c6x0Messages[2][0]);
  // can3.write(c6x0Messages[2][1]);
}


c620CAN::c620CAN() {

}


void c620CAN::init(uint8_t motorId, uint8_t tempCanBusNum) {
  canBusNum = tempCanBusNum;
  id = motorId;
  byteNum = id - 1;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr = &c6x0Messages[canBusNum-1][0];
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } else {
    sendMsgPtr = &c6x0Messages[canBusNum-1][1];
    sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
  }
}

void c620CAN::setPower(float power) {
  if (power > 1.0)
  {
    power = 1.0;
  } else if (power < -1.0) {
    power = -1.0;
  }
  
  int16_t newPower = (int16_t)(power * 16384);
  // Serial.print("sending: ");
  // Serial.println(newPower);

  byte byteOne = highByte(newPower);
  byte byteTwo = lowByte(newPower);
  sendMsgPtr->buf[byteNum << 1] = byteOne;
  sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
  // switch (canBusNum) {
  //   case 1:
  //     can1.write(*sendMsgPtr);
  //     break;
  //   case 2:
  //     can2.write(*sendMsgPtr);
  //     break;
  //   case 3:
  //     can3.write(*sendMsgPtr);
  //     break;
  // }
  updateMotor();
}

void c620CAN::updateMotor() {
    CAN_message_t *recMsg = &canRecieveMessages[canBusNum - 1][id - 1];
    
    uint16_t tempAngle;
    tempAngle = recMsg->buf[0];
    tempAngle = tempAngle << 8;
    tempAngle = tempAngle | recMsg->buf[1];
    angle = map(tempAngle, 0, 8191, 0, 36000) / 100.0;

    rpm = recMsg->buf[2];
    rpm = rpm << 8;
    rpm = rpm | recMsg->buf[3];

    torque = recMsg->buf[4];
    torque = torque << 8;
    torque = torque | recMsg->buf[5];

    temp = recMsg->buf[6];
}



c610Enc::c610Enc() {
  
}


void c610Enc::init(short tempID, uint8_t tempCanBusNum, uint8_t encPin) {
  canBusNum = tempCanBusNum;
  id = tempID;
  byteNum = id - 1;

  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr = &c6x0Messages[canBusNum-1][0];
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } 
  else {
    sendMsgPtr = &c6x0Messages[canBusNum - 1][1];
    sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
  }

  //On the Teensy 4.1 encPin can be pin 0-9,22-25,28,29,33,36,37,42-47, 48-50(dups),51, 52-53 (dups), 54
  if ((encPin >= 0 && encPin <= 9) || (encPin >= 22 && encPin <= 25) || encPin == 28 || encPin == 29 || encPin == 33 || encPin == 36 || encPin == 37 || (encPin >= 42 && encPin <= 54))
  {
    inPin = encPin;
    pinMode(inPin, INPUT); //set the pin used to measure the encoder to be an input
  } else {
    inPin = -1;
  }

  freq.begin(inPin, FREQMEASUREMULTI_MARK_ONLY);
}

void c610Enc::setPower(float power) {
  if (power > 1.0)
  {
    power = 1.0;
  } else if (power < -1.0) {
    power = -1.0;
  }
  
  int16_t newPower = (int16_t)(power * 16384);
  byte byteOne = highByte(newPower);
  byte byteTwo = lowByte(newPower);
  sendMsgPtr->buf[byteNum << 1] = byteOne;
  sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
  // Serial.println(newPower);
  // Serial.print(byteOne,HEX);
  // Serial.print(", ");
  // Serial.println(byteTwo, HEX);
  // switch (canBusNum) {
  //   case 1:
  //     can1.write(*sendMsgPtr);
  //     break;
  //   case 2:
  //     can2.write(*sendMsgPtr);
  //     break;
  //   case 3:
  //     can3.write(*sendMsgPtr);
  //     break;
  // }
  updateMotor();
}

//investigate using interrupts to handle keeping the angle up to date instead of getting into a while loop
//What is the cost of the while loop vs having constant interrupts
//what is the teensy's interrupt capabilities?
float c610Enc::getAngle() { 
  while (freq.available() > 2)  //burn through buffer of values in freq
  {
    freq.read();
  }
  float temp = map(round(freq.countToNanoseconds(freq.read())/1000), 1, 1024, 0, 36000) / 100.0;

  if (temp >= 0.0 && temp <= 360.0)
  {
    angle = temp;
  }

  return angle;
}

void c610Enc::updateMotor() {
    CAN_message_t *recMsg = &canRecieveMessages[canBusNum - 1][id - 1];
    // angle = recMsg->buf[0];
    // angle = angle << 8;
    // angle = angle | recMsg->buf[1];

    rpm = recMsg->buf[2];
    rpm = rpm << 8;
    rpm = rpm | recMsg->buf[3];

    torque = recMsg->buf[4];
    torque = torque << 8;
    torque = torque | recMsg->buf[5];

    temp = recMsg->buf[6];
}
