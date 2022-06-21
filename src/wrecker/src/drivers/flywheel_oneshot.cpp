#include "flywheel_oneshot.h"

flywheel_oneshot::flywheel_oneshot() {

}

void flywheel_oneshot::init(byte tempPinNum) {
    pinNum = tempPinNum;
    esc.attach(pinNum);

    esc.writeMicroseconds(125);
}

void flywheel_oneshot::setPower(float newPower) {
    if (newPower > 1.0)
    {
        newPower = 1.0;
    } else if (newPower < 0.0) {
        newPower = 0.0;
    }
    int sendPower = 125 + (newPower * 125);
    esc.writeMicroseconds(sendPower);
    // Serial.println(sendPower);
}