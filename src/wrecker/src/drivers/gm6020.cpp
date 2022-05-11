#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "gm6020.h"

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

CAN_message_t gm6020Messages[3][2];

gm6020::gm6020() {
  
}

void gm6020::init(short tempID, uint8_t tempCanBusNum){
  id = tempID;
  byteNum = id - 1;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr = &gm6020Messages[canBusNum-1][0];
    sendMsgPtr->id = 0x2FF;   //ID for all gm6020s 4-7
  } else {
    sendMsgPtr = &gm6020Messages[canBusNum-1][1];
    sendMsgPtr->id = 0x1FF;   //ID for all gm6020s 0-3
  }
}

void gm6020::setPower(float power) {
    short newPower = (short)(power * MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[byteNum] = byteOne;
    sendMsgPtr->buf[byteNum + 1] = byteTwo;
    switch (canBusNum) {
        case 1:
          can1.write(*sendMsgPtr);
          break;
        case 2:
          can2.write(*sendMsgPtr);
          break;
        case 3:
          can3.write(*sendMsgPtr);
          break;
    }
}
