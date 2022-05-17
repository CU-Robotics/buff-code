#include "flywheel.h"

flywheel::flywheel() {

}

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    pinMode(pinNum, OUTPUT);
    analogWriteFrequency(pinNum, 16000);
}

void setPower(float newPower) {
    if (newPower > 1)
    {
        newPower = 1;
    } else if (newPower < 0) {
        newPower = 0;
    }
    int sendPower = 0;
    sendPower = map(newPower * 1000, 0, 1000, 0, 255);
    analogWrite(pinNum, sendPower);
}