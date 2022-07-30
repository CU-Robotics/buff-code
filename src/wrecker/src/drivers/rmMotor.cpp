#include "rmMotor.h"

short rmMotor::getTorque() {
    return torque;
}

short rmMotor::getRpm() {
    return rpm;
}

float rmMotor::getAngle() {
    return angle;
}

byte rmMotor::getTemp() {
    return temp;
}

/*
format of data sent from motors over CAN

0 angle high byte
1 anlge low byte
2 speed high byte
3 speed low byte
4 torque high byte
5 torque low byte
6 temp
*/
// void rmMotor::updateMotor(CAN_message_t* recMsg) {
//     angle = recMsg->buf[0];
//     angle = angle << 8;
//     angle = angle | recMsg->buf[1];

//     rpm = recMsg->buf[2];
//     rpm = rpm << 8;
//     rpm = rpm | recMsg->buf[3];

//     torque = recMsg->buf[4];
//     torque = torque << 8;
//     torque = torque | recMsg->buf[5];

//     temp = recMsg->buf[6];
// }

// void rmMotor::updateMotor(short newTorque, short newRpm, short newAngle, byte newTemp) {
//     torque = newTorque;
//     rpm = newRpm;
//     angle = newAngle;
//     temp = newTemp;
// }


// void rmMotor::updateMotor() {
//     CAN_message_t *recMsg = canRecieveMessages[canBusNum - 1][]
//     angle = recMsg->buf[0];
//     angle = angle << 8;
//     angle = angle | recMsg->buf[1];

//     rpm = recMsg->buf[2];
//     rpm = rpm << 8;
//     rpm = rpm | recMsg->buf[3];

//     torque = recMsg->buf[4];
//     torque = torque << 8;
//     torque = torque | recMsg->buf[5];

//     temp = recMsg->buf[6];
// }