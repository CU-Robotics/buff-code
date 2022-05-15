#include "flywheel.h"

flywheel::flywheel() {

}

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    pinMode(pinNum, OUTPUT);
    analogWriteFrequency(pinNum, 16000);
}

void setPower(float newPower) {
    
}