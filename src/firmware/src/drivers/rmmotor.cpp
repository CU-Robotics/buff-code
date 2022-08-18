#include <Arduino.h>

#include "rmmotor.h"


RMMotor::RMMotor() {
    motor_id = -1;
    can_id = -1;
}

RMMotor::RMMotor(CAN_message_t* send, CAN_message_t* rec, int idx, int canid, int motorid){
    id = idx;
	motor_id = motorid;
	can_id = canid;
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

    // Serial.print("Setting power ");
    // Serial.println(power);

    int16_t newPower = (int16_t)(power * 16384);
    byte byteOne = highByte(newPower);
    byte byteTwo = lowByte(newPower);
    sendMsgPtr->buf[motor_id << 1] = byteOne;
    sendMsgPtr->buf[(motor_id << 1) + 1] = byteTwo;
}

int RMMotor::read(HIDBuffer* buffer) {
    static int val = 0.0;
    if (motor_id != -1){
        // add buffer3 to filter these values
        // Serial.println("Sending motor value");
        uint16_t test = map((sin(val / 100)), -1, 1, 0, 62800);
        val += 1;

        if (!buffer->check_of(11)) {
            buffer->put('T');
            buffer->put('T');
            buffer->put(6);
            buffer->put(id);
            buffer->put_u16(test); //map((recMsgPtr->buf[0] << 8) | recMsgPtr->buf[1], 0, 8191, 0, 36000));
            buffer->put_u16((recMsgPtr->buf[2] << 8) | recMsgPtr->buf[3]);
            buffer->put_u16((recMsgPtr->buf[4] << 8) | recMsgPtr->buf[5]);
            return 1;
        }
        else {
            return 0;
        }
    }
	
    return -1;
}

void new_motor(Motor_LUT* minfo, int idx, int can_id, int motor_id, int motor_type){

    CAN_message_t* recMsgPtr = &minfo->canReceiveMessages[can_id - 1][motor_id + 4];
    CAN_message_t* sendMsgPtr;

    int tmp_can = can_id;
    int tmp_mot = motor_id;

    // Serial.print("New motor "); Serial.print(tmp_can); Serial.print(" "); Serial.print(tmp_mot); Serial.print(" "); Serial.println(idx);

    switch (motor_type) {
        case 0:
            sendMsgPtr = &minfo->canSendMessages[can_id - 1][1 - motor_id / 4];
            sendMsgPtr->id = (motor_id / 4 >= 1)? 0x1FF:0x200;
            motor_id = motor_id % 4;
            break;

        case 1:
            sendMsgPtr = &minfo->canSendMessages[can_id + 2][1 - motor_id / 4];
            sendMsgPtr->id = (motor_id / 4 >= 1)? 0x2FF:0x1FF;
            motor_id = motor_id % 4;
            break;
    }
    minfo->motors[idx] = RMMotor(sendMsgPtr, recMsgPtr, idx, tmp_can, tmp_mot);
}

void read_motors(Motor_LUT* minfo, HIDBuffer* buffer) {
    static int seek_id = 0;
    unsigned long timeout = micros();

    while (micros() - timeout < 50) {
        if (minfo->motors[seek_id].id != -1) {
            if (minfo->motors[seek_id].read(buffer) == 0){
                // Serial.println("Read motor");
                break;
            }
        }
        seek_id++;

        if (seek_id >= 24){
            seek_id = 0;
            break;
        }
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