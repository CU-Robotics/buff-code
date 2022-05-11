#include "c620.h"

c620CAN::c620CAN() {

}

// c620CAN::c620CAN(short motorId, CAN_message_t* mySendMsgPtr){
//   id = motorId;
//   byteNum = id - 1;
//   sendMsgPtr = mySendMsgPtr;
//   if(byteNum > 3) {
//     byteNum -= 4;
//     sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
//   } else {
//     sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
//   }
// }

void c620CAN::init(short motorId, CAN_message_t* msg) {
  id = motorId;
  byteNum = id - 1;
  sendMsgPtr = msg;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } else {
    sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
  }
}

void c620CAN::setPower(float power) {
    short newPower = (short)(power * MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[byteNum << 1] = byteOne;
    sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
}




c610Enc::c610Enc() {
  
}

// c610Enc::c610Enc(short tempID, CAN_message_t* mySendMsgPtr, uint8_t encPin) {
//   id = tempID;
//   byteNum = id - 1;
//   sendMsgPtr = mySendMsgPtr;
//   if(byteNum > 3) {
//     byteNum -= 4;
//     sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
//   } else {
//     sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
//   }

  
//   //On the Teensy 4.1 encPin can be pin 0-9,22-25,28,29,33,36,37,42-47, 48-50(dups),51, 52-53 (dups), 54
//   if ((encPin >= 0 && encPin <= 9) || (encPin >= 22 && encPin <= 25) || encPin == 28 || encPin == 29 || encPin == 33 || encPin == 36 || encPin == 37 || (encPin >= 42 && encPin <= 54))
//   {
//     inPin = encPin;
//     pinMode(inPin, INPUT); //set the pin used to measure the encoder to be an input
//   } else {
//     inPin = -1;
//   }

//   freq.begin(inPin, FREQMEASUREMULTI_MARK_ONLY);
// }

void c610Enc::init(short tempID, CAN_message_t* msg, uint8_t encPin) {
  id = tempID;
  byteNum = id - 1;
  sendMsgPtr = msg;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } else {
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
    short newPower = (short)(power * MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[byteNum << 1] = byteOne;
    sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
}

//investigate using interrupts to handle keeping the angle up to date instead of getting into a while loop
//What is the cost of the while loop vs having constant interrupts
//what is the teensy's interrupt capabilities?
float c610Enc::getAngle() { 
  while (freq.available() > 2)  //burn through buffer of values in freq
  {
    freq.read();
  }
  
  angle = map(round(freq.countToNanoseconds(freq.read())/1000), 1, 1024, 0, 360);
  
  return angle;
}