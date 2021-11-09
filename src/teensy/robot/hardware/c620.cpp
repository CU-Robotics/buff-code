#ifndef CONSTANTS_H
#include "../../../constants.h"
#endif

#include "c620.h"


c620::c620(short tempID, CAN_message_t *msg) {
    id = tempID;
    *sendMsg = msg;
    byteNum = id - 1;
    if(byteNum > 3) {
        byteNum -= 4;
    }
}

c620::setPower(float power) {
    short newPower = (short)(power * C620_MAX_VALUE);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsg.buf(byteNum) = highByte;
    sendMsg.buf(byteNum + 1) = lowByte;
}