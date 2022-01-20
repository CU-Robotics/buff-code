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

c620PWM::c620PWM(uint8_t input, uint8_t output) {
  //On the Teensy 4.1 input can be pin 0-9,22-25,28,29,33,36,37,42-47, 48-50(dups),51, 52-53 (dups), 54
  if ((input >= 0 && input <= 9) || (input >= 22 && <= 25) || input == 28 || input == 29 || input == 33 || input == 36 || input == 37 || (input >= 42 && input <= 54))
  {
    inPin = input;
  } else {
    inPin = -1;
  }
  
  if ((output >= 0 && output <= 15) || output == 18 || output == 19 || (output >= 22 && output <= 25) || output == 28 || output == 29 || output == 33 || output == 36 || output == 37 || (output >= 42 && output <= 47) || output == 51 || output == 54)
  {
    outPin = output;
  } else {
    outPin = -1;
  }
  
}