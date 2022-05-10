#include "reciever_serial.h"

recSerial::recSerial() {

}

bool recSerial::parseMessage(byte buf[14]) {
    message.rStickH = buf[1] & 0b00000111;
    message.rStickH = message.rStickH << 8;
    message.rStickH = message.rStickH | buf[0];

    message.rStickV = buf[2] & 0b00111111;
    message.rStickV = message.rStickV << 8;
    message.rStickV = message.rStickV | (buf[1] & 0b11111000);

    message.lStickV = buf[4] & 0b00000001;
    message.lStickV = message.lStickV << 8;
    message.lStickV = message.lStickV | buf[3];
    message.lStickV = (message.lStickV << 8) | (buf[2] & 0b11000000);
}

int recSerial::getRightStickVertical() {
    return message.rStickH;
}

int recSerial::getRightStickHorizontal() {
    return message.rStickV;
}

int recSerial::getLeftStickHorizontal() {
    return message.lStickH;
}

int recSerial::getLeftStickVertical() {
    return message.lStickV;
}