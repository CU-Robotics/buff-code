#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_can.h"


BuffCan::BuffCan(){
    can1.begin();
    can2.begin();
    can1.setBaudRate(1000000);
    can2.setBaudRate(1000000);


    input[0][0].id = 0x200;
    input[0][1].id = 0x1FF;
    input[0][2].id = 0x2FF;

    input[1][0].id = 0x200;
    input[1][1].id = 0x1FF;
    input[1][2].id = 0x2FF;

}

void BuffCan::set(int cid, int id, int offset, byte high, byte low){

    // Serial.println("New Can device");
    // if (millis() - d_t < 10){
    //     return; // Maybe here just wait
    // }    
    offset *= 2;
    input[cid][id].buf[offset] = high;
    input[cid][id].buf[offset+1] = low;

    d_t = millis();
}

void BuffCan::write(){
    for (int i = 0; i < 3; i++){
        can1.write(input[0][i]);
        can2.write(input[1][i]);
    }

    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 8; k++){
                input[i][j].buf[k] = 0; 
            }
        }
    }
}

void print_can_message(CAN_message_t msg) {
    Serial.println("======");
    Serial.print("id: "); Serial.println(msg.id, HEX);
    Serial.print(map((msg.buf[0] << 8) | msg.buf[1], 0, 8191, -180, 180));
    Serial.print("\t");
    Serial.print(uint16_t(msg.buf[2]) << 8 | uint16_t(msg.buf[3]));
    Serial.print("\t");
    Serial.print(uint16_t(msg.buf[4]) << 8 | uint16_t(msg.buf[5]));
    Serial.print("\t");
    Serial.println(msg.buf[6]);
    
}

void BuffCan::read(){

    CAN_message_t tmp;

    can2.read(tmp);
    if (tmp.id == 0x202) {
        print_can_message(tmp);
    }
    
}
