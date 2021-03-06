#include "c620.h"

c620CAN::c620CAN(short motorId, CAN_message_t* mySendMsgPtr){
  id = motorId;
  byteNum = id - 1;
  sendMsgPtr = mySendMsgPtr;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } else {
    sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
  }
}

void c620CAN::setPower(float power) {
    short newPower = (short)(power * C620_MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[byteNum << 1] = byteOne;
    sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
}

c620PWM::c620PWM(uint8_t input, uint8_t output) {
  //On the Teensy 4.1 input can be pin 0-9,22-25,28,29,33,36,37,42-47, 48-50(dups),51, 52-53 (dups), 54
  if ((input >= 0 && input <= 9) || (input >= 22 && input <= 25) || input == 28 || input == 29 || input == 33 || input == 36 || input == 37 || (input >= 42 && input <= 54))
  {
    inPin = input;
  } else {
    inPin = -1;
  }

  freq.begin(inPin, FREQMEASUREMULTI_MARK_ONLY);
  
  if ((output >= 0 && output <= 15) || output == 18 || output == 19 || (output >= 22 && output <= 25) || output == 28 || output == 29 || output == 33 || output == 36 || output == 37 || (output >= 42 && output <= 47) || output == 51 || output == 54)
  {
    outPin = output;
  } else {
    outPin = -1;
  }
  
  pinMode(outPin, OUTPUT);
  analogWriteFrequency(outPin, 500);
  analogWriteResolution(15);
}

//useful conversions of uS to pwm dutycycle
// 1000-2000
// 16378-32757

// 1000-1500
// 16378-24567

// 1500-2000
// 24567-32757

// 1080-1480
// 17688-24240

// 1520-1920
// 24894-31447

// 0-1
// 0-16.3785

// 0-20
// 0-327

// 0-80
// 0-1310

void c620PWM::setPower(float power) {
  unsigned int outputVal = 0;
  if(power > 1) { //clamp power to -1 to 1
    power = 1;
  } else if (power < -1) {
    power = -1;
  }

  if (power > 0) {
    outputVal = map(power, 0, 1, 24894, 31447);
  } else if (power < 0) {
    outputVal = map(power, -1, 0, 17688, 24240);
  } else {
    outputVal = 1500;
  }
  
  // outputVal = map(power, -1, 1, 16378, 32757);  //this approach is ignorant to the deadzones on the motor's PWM recieve

  analogWrite(outPin, outputVal);
}

float c620PWM::getAngle() {
  while (freq.available() > 2)  //burn through buffer of values in freq
  {
    freq.read();
  }
  
  angle = map(round(freq.countToNanoseconds(freq.read())/1000), 1, 1024, 0, 360);
  
  return angle;
}


c610Enc::c610Enc(short motorId, CAN_message_t* mySendMsgPtr, uint8_t encPin) {
  id = motorId;
  byteNum = id - 1;
  sendMsgPtr = mySendMsgPtr;
  if(byteNum > 3) {
    byteNum -= 4;
    sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
  } else {
    sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
  }

  //On the Teensy 4.1 input can be pin 0-9,22-25,28,29,33,36,37,42-47, 48-50(dups),51, 52-53 (dups), 54
  if ((encPin >= 0 && encPin <= 9) || (encPin >= 22 && encPin <= 25) || encPin == 28 || encPin == 29 || encPin == 33 || encPin == 36 || encPin == 37 || (encPin >= 42 && encPin <= 54))
  {
    inPin = encPin;
    // pinMode(inPin, INPUT); //set the pin used to measure the encoder to be an input
  } else {
    inPin = -1;
  }

  freq.begin(inPin, FREQMEASUREMULTI_MARK_ONLY);
}

void c610Enc::setPower(float power) {
    short newPower = (short)(power * C620_MAX_VALUE);
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

void c610Enc::updateMotor(CAN_message_t* recMsg) {
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