#include "flywheel.h"

flywheel::flywheel() {

}

void flywheel::initCal(byte tempPinNum) {
    pinNum = tempPinNum;
    esc.attach(pinNum);

    esc.writeMicroseconds(1900);
    delay(3500);
    esc.writeMicroseconds(1100);
    delay(3500);
    // pinMode(pinNum, OUTPUT);
    // analogWriteFrequency(pinNum, 250);

    // analogWriteResolution(10);
    
    // analogWrite(pinNum, 511);   //throttle high
    // delay(4000);
    // analogWrite(pinNum, 255);   //throttle low
    // delay(4000);
    // analogWrite(pinNum, 384);   //mid throttle
    // delay(3500);
}

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    esc.attach(pinNum);

    esc.writeMicroseconds(1900);
    delay(100);
    esc.writeMicroseconds(1100);
    delay(100);

    // pinMode(pinNum, OUTPUT);
    // analogWriteFrequency(pinNum, 250);

    // analogWriteResolution(10);

    // analogWrite(pinNum, 512);   //throttle high
    // delay(100);
    // analogWrite(pinNum, 256);   //throttle low
    // delay(100);
    // analogWrite(pinNum, 256);   //low
}

void flywheel::setPower(float newPower) {
    if (newPower > 1)
    {
        newPower = 1;
    } else if (newPower < 0) {
        newPower = 0;
    }
    int sendPower = 1000 + (newPower * 1000);
    // sendPower = map(newPower * 1000, 0, 1000, 256, 512);
    // analogWrite(pinNum, sendPower);
    esc.writeMicroseconds(sendPower);
    Serial.println(sendPower);
}