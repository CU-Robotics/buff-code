#include "dr16.h"

dr16::dr16() {
    
}

void dr16::init(DriverInput *tempInput) {
    input = tempInput;
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);  //reciever serial
}

void dr16::update() {
    // while(Serial5.available()) {
    //     Serial5.read();
    // }
    // while(Serial5.available() < 19) {
        
    // }

    if (Serial5.available() > numBytes)     //if there are more bytes in the serial buffer, update the number of bytes and the lastTime variable
    {
        numBytes = Serial5.available();
        lastTime = micros();
    }

    if (micros() - lastTime > 150)  //if more than 150 microseconds has passed since the last byte recieved then frame has probably ended
    {
        if (Serial5.available() % 18 != 0)  //if the number of bytes is not divisible by 18 then there is a mangled frame and all data should be thrown out
        {
            while (Serial5.available())
            {
                Serial5.read();
            }
            numBytes = 0;
        }
        
    }
    

    if(Serial5.available() >= 18) {
        Serial5.readBytes(buf, 18);

        numBytes = 0;

        //debugging
        // for (int i = 0; i < 18; i++)
        // {
        //     for (int j = 7; j >= 0; j--)
        //     {
        //         Serial.print(bitRead(buf[i], j));
        //     }
        //     Serial.print(" ");
        // }
        // Serial.println();
        
        input->leftStickX = ((buf[1] & 0b00000111) << 8) | buf[0];
        input->leftStickY = ((buf[2] & 0b11111100) << 5) | ((buf[1] & 0b11111000) >> 3);
        input->rightStickX = (((buf[4] & 0b00000001) << 10) | (buf[3] << 2)) | (buf[2] & 0b00000011);
        input->rightStickY = ((buf[5] & 0b00001111) << 7) | (buf[4] & 0b11111110);

        input->s1 = ((buf[5] & 0b00110000) >> 4);
        input->s2 = ((buf[5] & 0b11000000) >> 6);

        // Serial.println(input->S1, BIN);

        //first byte of keyboard
        input->w = buf[14] & 0b00000001;
        input->s = buf[14] & 0b00000010;
        input->a = buf[14] & 0b00000100;
        input->d = buf[14] & 0b00001000;
        input->shift = buf[14] & 0b00010000;
        input->ctrl = buf[14] & 0b00100000;
        input->q = buf[14] & 0b01000000;
        input->e = buf[14] & 0b10000000;
        
        //second byte of keyboard
        input->r = buf[15] & 0b00000001;
        input->f = buf[15] & 0b00000010;
        input->g = buf[15] & 0b00000100;
        input->z = buf[15] & 0b00001000;
        input->x = buf[15] & 0b00010000;
        input->c = buf[15] & 0b00100000;
        input->v = buf[15] & 0b01000000;
        input->b = buf[15] & 0b10000000;


        //mouse
        input->mouseX = (buf[6] << 8) | buf[7];
        input->mouseY = (buf[8] << 8) | buf[9];
        input->mouseZ = (buf[10] << 8) | buf[11];
        input->mouseLeft = buf[12];
        input->mouseRight = buf[13];


        //remote wheel
        input->remoteWheel = (buf[17] << 8) | buf[16];

        // Serial.println(input->remoteWheel);
        // Serial.println(input->CH3); 
        // Serial.println();
        // Serial.print(input->r);
        // Serial.print(", ");
        // Serial.print(input->f);
        // Serial.print(", ");
        // Serial.print(input->g);
        // Serial.print(", ");
        // Serial.print(input->z);
        // Serial.print(", ");
        // Serial.print(input->x);
        // Serial.print(", ");
        // Serial.print(input->c);
        // Serial.print(", ");
        // Serial.print(input->v);
        // Serial.print(", ");
        // Serial.print(input->b);

        // Serial.print(input->s1);
        // Serial.print(", ");
        // Serial.print(input->s2);

        // Serial.print(input->mouseLeft);
        // Serial.print(", ");
        // Serial.print(input->mouseRight);
        // Serial.println();
    }
}