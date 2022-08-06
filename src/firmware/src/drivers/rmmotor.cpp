#include <Arduino.h>

#include "rmmotor.h"


RMMotor::RMMotor() {
    id = -1;
    byte_num = -1;
    canBusID = -1;
}

RMMotor::RMMotor(CAN_message_t* send, CAN_message_t* rec, int idx, unsigned int canBusID, unsigned int byteNum){
    id = idx;
	byte_num = byteNum;
	canBusID = canBusID;
    sendMsgPtr = send;
    recMsgPtr = rec;
}

void RMMotor::setPower(float power) {
	if (power > 1.0){
        power = 1.0;
    } 
    else if (power < -1.0){
        power = -1.0;
    }

    int16_t newPower = (int16_t)(power * 16384);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[byte_num << 1] = byteOne;
    sendMsgPtr->buf[(byte_num << 1) + 1] = byteTwo;
}

int RMMotor::read(HIDBuffer* buffer) {

    if (id != -1){
        // add buffer3 to filter these values

        if (!buffer->check_of(11)) {
            buffer->put('M');
            buffer->put('M');
            buffer->put(6);
            buffer->put(id);
            buffer->put(1);
            buffer->put_u16(map((recMsgPtr->buf[0] << 8) | recMsgPtr->buf[1], 0, 8191, 0, 36000));
            buffer->put_u16((recMsgPtr->buf[2] << 8) | recMsgPtr->buf[3]);
            buffer->put_u16((recMsgPtr->buf[4] << 8) | recMsgPtr->buf[5]);
            return 1;
        }
        else {
            return 0;
        }
    }
	
    return 1;
}

void new_motor(Motor_LUT* minfo, int idx, unsigned int canBusID, unsigned int byte_num, unsigned int motor_type){

    CAN_message_t* recMsgPtr = &minfo->canReceiveMessages[canBusID - 1][byte_num + 4];
    CAN_message_t* sendMsgPtr;

    switch (motor_type) {
        case 0:
            sendMsgPtr = &minfo->canSendMessages[canBusID - 1][1 - byte_num / 4];
            sendMsgPtr->id = (byte_num / 4 >= 1)? 0x1FF:0x200;
            byte_num = byte_num % 4;
            minfo->motors[idx] = RMMotor(sendMsgPtr, recMsgPtr, idx, canBusID, byte_num);
            break;

        case 1:
            sendMsgPtr = &minfo->canSendMessages[canBusID + 2][1 - byte_num / 4];
            sendMsgPtr->id = (byte_num / 4 >= 1)? 0x2FF:0x1FF;
            byte_num = byte_num % 4;
            minfo->motors[idx] = RMMotor(sendMsgPtr, recMsgPtr, idx, canBusID, byte_num);
            break;
    }
}

void initCAN(Motor_LUT* minfo){
    minfo->can1.begin();
    minfo->can2.begin();

    minfo->can1.setBaudRate(1000000);
    minfo->can2.setBaudRate(1000000);
}


void writeCAN(Motor_LUT* minfo){
    minfo->can1.write(minfo->canSendMessages[0][0]);
    minfo->can1.write(minfo->canSendMessages[0][1]);
    minfo->can1.write(minfo->canSendMessages[1][0]);
    minfo->can1.write(minfo->canSendMessages[1][1]);
    minfo->can1.write(minfo->canSendMessages[2][0]);
    minfo->can1.write(minfo->canSendMessages[2][1]);
    //
    minfo->can2.write(minfo->canSendMessages[3][0]);
    minfo->can2.write(minfo->canSendMessages[3][1]);
    minfo->can2.write(minfo->canSendMessages[4][0]);
    minfo->can2.write(minfo->canSendMessages[4][1]);
    minfo->can2.write(minfo->canSendMessages[5][0]);
    minfo->can2.write(minfo->canSendMessages[5][1]);
}