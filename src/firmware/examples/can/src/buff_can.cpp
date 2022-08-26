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

BuffCan::BuffCan(){
    can.begin();
    can.setBaudRate(1000000);


    input[0].id = 0x200;
    input[1].id = 0x1FF;
    input[2].id = 0x2FF;
}

void BuffCan::set(int mid, byte* msg){
    // Serial.print("Setting can "); Serial.println(mid);
    for (int i = 0; i < 8; i++) {
        input[mid].buf[i] = msg[i];
    }

    d_t = millis();
}

void BuffCan::write(){
    for (int i = 0; i < 3; i++){
        can.write(input[i]);
    }
}

void BuffCan::read(){

    CAN_message_t tmp;
    can.read(tmp);
    print_can_message(&tmp);
    int offset = ((tmp.id - 0x1FF) / 4) * 8;
    Serial.print("can offset: ");
    Serial.println(offset);
}
