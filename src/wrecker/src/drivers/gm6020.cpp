#include <FlexCAN_T4.h>

#include "gm6020.h"

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

extern CAN_message_t canRecieveMessages[3][11];

CAN_message_t gm6020Messages[3][2];

gm6020::gm6020() {
  
}

void gm6020::init(short tempID, uint8_t tempCanBusNum){
  canBusNum = tempCanBusNum;
  id = tempID;
  byteNum = id - 1;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr = &gm6020Messages[canBusNum-1][0];
    sendMsgPtr->id = 0x2FF;   //ID for all gm6020s 5-7
  } else {
    sendMsgPtr = &gm6020Messages[canBusNum-1][1];
    sendMsgPtr->id = 0x1FF;   //ID for all gm6020s 0-3
  }
}

void gm6020::setPower(float power) {
  if (power > 1.0)
  {
    power = 1.0;
  } else if (power < -1.0) {
    power = -1.0;
  }
  
  int16_t newPower = (int16_t)(power * 30000);
  // Serial.print("newPower: ");
  // Serial.println(newPower);
  byte byteOne = highByte(newPower);
  byte byteTwo = lowByte(newPower);
  sendMsgPtr->buf[byteNum << 1] = byteOne;
  sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
  switch (canBusNum) {
    case 1:
      can1.write(*sendMsgPtr);
      break;
    case 2:
      can2.write(*sendMsgPtr);
      // Serial.println("writing to can2");
      break;
    case 3:
      can3.write(*sendMsgPtr);
      break;
  }
  updateMotor();
}

void gm6020::updateMotor() {
  CAN_message_t *recMsg = &canRecieveMessages[canBusNum - 1][id + 3];
  
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