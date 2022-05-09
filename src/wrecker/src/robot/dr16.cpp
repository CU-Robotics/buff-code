#include "dr16.h"

dr16::dr16() {
    
}

void dr16::init(RobotInput *tempInput) {
    input = tempInput;
    Serial5.begin(115200, SERIAL_8E1_RXINV_TXINV);  //reciever serial
}

void dr16::update() {
    while(Serial5.available()) {
        Serial5.read();
    }
    while(Serial5.available() < 19) {
        
    }
    if(Serial5.available() >= 18) {
        Serial5.readBytes(buf, 18);

        //debugging
        for (int i = 0; i < 18; i++)
        {
            for (int j = 7; j >= 0; j--)
            {
                Serial.print(bitRead(buf[i], j));
            }
            Serial.print(" ");
        }
        Serial.println();
        

        // input->S1 = (buf[5] & 0b00011000 >> 3);
        // input->S1 = buf[5];

        // Serial.println(input->S1, BIN);

        input->w = buf[14] & 0b00000001;
        input->s = buf[14] & 0b00000010;
        input->a = buf[14] & 0b00000100;
        input->d = buf[14] & 0b00001000;
        input->f = buf[15] & 0b00000010;
    }
}