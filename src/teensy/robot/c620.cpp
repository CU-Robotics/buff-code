#ifndef CONSTANTS_H
#include "constants.h"
#endif

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "c620.h"



c620::c620(short tempID, CAN_message_t* msg) : sendMsg(*msg) {
  //c620::sendMsg;
//  sendMsg = *msg;
  id = tempID;
  byteNum = id - 1;
  if(byteNum > 3) {
      byteNum -= 4;
  }
  sendMsg.id = 2;
}

void c620::setPower(float power) {
    short newPower = (short)(power * C620_MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsg.buf[byteNum] = byteOne;
    sendMsg.buf[byteNum + 1] = byteTwo;
}
