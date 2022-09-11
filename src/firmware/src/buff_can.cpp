#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_can.h"


void print_can_message(CAN_message_t* msg) {
    Serial.println("======");
    Serial.print("id: "); Serial.println(msg->id - 0x200, HEX);
    Serial.print("\t");
    Serial.print(msg->buf[0]);
    Serial.print("\t");
    Serial.print(msg->buf[1]);

    Serial.print("\t");
    Serial.print(msg->buf[2]);
    Serial.print("\t");
    Serial.print(msg->buf[3]);
    Serial.print("\t");
    Serial.print(msg->buf[4]);
    Serial.print("\t");
    Serial.print(msg->buf[5]);
    Serial.print("\t");
    Serial.print(msg->buf[6]);
    Serial.print("\t");
    Serial.println(msg->buf[7]);  
}

void prettyprint_can_message(CAN_message_t* msg) {
    Serial.println("======");
    Serial.print("id: "); Serial.println(msg->id - 0x200, HEX);
    Serial.print("\t");
    Serial.print(map((uint16_t(msg->buf[0]) << 8) | uint16_t(msg->buf[1]), 0, 8191, 0, 360));
    Serial.print("\t");
    Serial.print((int16_t(msg->buf[2]) << 8) | int16_t(msg->buf[3]));
    Serial.print("\t");
    Serial.println((int16_t(msg->buf[2]) << 8) | int16_t(msg->buf[3]));
}

BuffCan::BuffCan(){
    can1.begin();
    can1.setBaudRate(1000000);
    can2.begin();
    can2.setBaudRate(1000000);

    for (int i = 0; i < 2; i++){
        input[i][0].id = 0x200;
        input[i][1].id = 0x1FF;
        input[i][2].id = 0x2FF;
    }
}

void BuffCan::zero_can() {
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 8; j++){
            input[0][i].buf[j] = 0.0;
            input[1][i].buf[j] = 0.0;
        }
    }
}

void BuffCan::write(){
    for (int i = 0; i < 3; i++){
        can1.write(input[0][i]);
        can2.write(input[1][i]);
    }
}

void BuffCan::set_input(int can, int mid, byte* msg){
    // Serial.print("Setting can "); Serial.println(mid);
    for (int i = 0; i < 8; i++) {
        input[can - 1][mid].buf[i] = msg[i];
    }

    d_t = millis();
}

int BuffCan::set_output(int msg_ctr, CAN_message_t* msg, byte* buff){
    int msg_id = msg->id - 0x200;

    if (msg_id == 255){
        return 0;
    }
    uint16_t angle = map((uint16_t(msg->buf[0]) << 8) | uint16_t(msg->buf[1]), 0, 8191, 0, 36000);

    buff[(msg_ctr * 8) + 1] = msg_id / 4;
    buff[(msg_ctr * 8) + 2] = ((msg_id % 4) * 6) + 1;

    buff[(msg_ctr * 8) + 3] = byte((angle && 0xFF00) >> 8);
    buff[(msg_ctr * 8) + 4] = byte(angle && 0xFF);

    buff[(msg_ctr * 8) + 5] = msg->buf[2];
    buff[(msg_ctr * 8) + 6] = msg->buf[3];

    buff[(msg_ctr * 8) + 7] = msg->buf[4];
    buff[(msg_ctr * 8) + 8] = msg->buf[5];

    return 1;
}

void BuffCan::read_can1(byte* buff){
    int msg_ctr = 0;
    CAN_message_t tmp;
    buff[0] = 1;

    while (msg_ctr < 4) {
        can1.read(tmp);
        msg_ctr += set_output(msg_ctr, &tmp, buff);
    }  
}

void BuffCan::read_can2(byte* buff){
    int msg_ctr = 0;
    CAN_message_t tmp;
    buff[0] = 2;

    while (msg_ctr < 4) {
        can2.read(tmp);
        prettyprint_can_message(&tmp);
        msg_ctr += set_output(msg_ctr, &tmp, buff);
    }  
}
