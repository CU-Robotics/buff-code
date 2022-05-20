#include "flywheel.h"

flywheel::flywheel() {

}

void flywheel::initCal(byte tempPinNum) {
    pinNum = tempPinNum;
    pinMode(pinNum, OUTPUT);
    analogWriteFrequency(pinNum, 250);

    analogWriteResolution(10);
    
    analogWrite(pinNum, 512);   //throttle high
    delay(4000);
    analogWrite(pinNum, 256);   //throttle low
    delay(4000);
    // analogWrite(pinNum, 384);   //mid throttle
    delay(3500);
}

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    pinMode(pinNum, OUTPUT);
    analogWriteFrequency(pinNum, 250);

    analogWriteResolution(10);

    analogWrite(pinNum, 512);   //throttle high
    delay(100);
    analogWrite(pinNum, 256);   //throttle low
    delay(100);
    // analogWrite(pinNum, 256);   //low
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
    Serial.println(sendPower);
}