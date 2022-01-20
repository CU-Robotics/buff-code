#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "c620.h"



c620CAN::c620CAN(short motorId, CAN_message_t* msg) : sendMsg(*msg) {
  id = motorId;
  byteNum = id - 1;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsg.id = 0x1FF;   //ID for all c620s 4-7
  } else {
  sendMsg.id = 0x200;   //ID for all c620s 0-3
  }
}

void c620CAN::setPower(float power) {
    short newPower = (short)(power * C620_MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsg.buf[byteNum] = byteOne;
    sendMsg.buf[byteNum + 1] = byteTwo;
}
