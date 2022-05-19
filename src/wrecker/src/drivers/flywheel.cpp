#include "flywheel.h"

flywheel::flywheel() {

}

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    pinMode(pinNum, OUTPUT);
    analogWriteFrequency(pinNum, 250);

    analogWriteResolution(10);

    analogWrite(pinNum, 512);
    delay(3500);
    analogWrite(pinNum, 256);
}

void flywheel::setPower(float newPower) {
    if (newPower > 1)
    {
        newPower = 1;
    } else if (newPower < 0) {
        newPower = 0;
    }
    int sendPower = 0;
    sendPower = map(newPower * 1000, 0, 1000, 256, 512);
    analogWrite(pinNum, sendPower);
}