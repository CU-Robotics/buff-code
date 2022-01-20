#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "gm6020.h"

gm6020::gm6020(short tempID, CAN_message_t* msg) : sendMsg(*msg) {
  id = motorId;
  byteNum = id - 1;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsg.id = 0x2FF;   //ID for all c620s 4-7
  } else {
    sendMsg.id = 0x1FF;   //ID for all c620s 0-3
  }
}

void gm6020::setPower(float power) {
    short newPower = (short)(power * GM6020_MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsg.buf[byteNum] = byteOne;
    sendMsg.buf[byteNum + 1] = byteTwo;
}
