#include "flywheel.h"

flywheel::flywheel() {

}

// void flywheel::initCal(byte tempPinNum) {
//     pinNum = tempPinNum;
//     esc.attach(pinNum);

//     esc.writeMicroseconds(1900);
//     delay(3500);
//     esc.writeMicroseconds(1100);
//     delay(3500);
// }

void flywheel::init(byte tempPinNum) {
    pinNum = tempPinNum;
    esc.attach(pinNum);

    esc.writeMicroseconds(1000);
}

void flywheel::setPower(float newPower) {
    // if (newPower > 1.0)
    // {
    //     newPower = 1.0;
    // } else if (newPower < 0.0) {
    //     newPower = 0.0;
    // }
    int sendPower = 1000 + (newPower * 1000);
    esc.writeMicroseconds(sendPower);
    // Serial.println(sendPower);
}