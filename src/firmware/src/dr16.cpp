#include "dr16.h"

DR16::DR16(){
    d_t = micros();
    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV); 
}

bool DR16::read(byte* buffer) {

    if (Serial5.available() < 18)
    {
        return false;
    }
    else if (Serial5.available() % 18 != 0) {
        Serial5.clear();
        return false;
    }

    if(Serial5.available() >= 18) {

    Serial5.readBytes(buffer, 18);
    d_t = micros();

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
        
        input->leftStickX = ((buffer[1] & 0b00000111) << 8) | buffer[0];
        input->leftStickY = ((buffer[2] & 0b11111100) << 5) | ((buffer[1] & 0b11111000) >> 3);
        input->rightStickX = (((buffer[4] & 0b00000001) << 10) | (buffer[3] << 2)) | (buf[2] & 0b00000011);
        input->rightStickY = ((buffer[5] & 0b00001111) << 7) | (buffer[4] & 0b11111110);

        input->s1 = ((buffer[5] & 0b00110000) >> 4);
        input->s2 = ((buffer[5] & 0b11000000) >> 6);

        // Serial.println(input->S1, BIN);

        //first byte of keyboard
        input->w = buffer[14] & 0b00000001;
        input->s = buffer[14] & 0b00000010;
        input->a = buffer[14] & 0b00000100;
        input->d = buffer[14] & 0b00001000;
        input->shift = buffer[14] & 0b00010000;
        input->ctrl = buffer[14] & 0b00100000;
        input->q = buffer[14] & 0b01000000;
        input->e = buffer[14] & 0b10000000;
        
        //second byte of keyboard
        input->r = buffer[15] & 0b00000001;
        input->f = buffer[15] & 0b00000010;
        input->g = buffer[15] & 0b00000100;
        input->z = buffer[15] & 0b00001000;
        input->x = buffer[15] & 0b00010000;
        input->c = buffer[15] & 0b00100000;
        input->v = buffer[15] & 0b01000000;
        input->b = buffer[15] & 0b10000000;


        //mouse
        input->mouseX = (int16_t)(buffer[6] | (buffer[7] << 8));
        input->mouseY = (int16_t)(buffer[8] | (buffer[9] << 8));
        input->mouseZ = (int16_t)(buffer[10] | (buffer[11] << 8));
        input->mouseLeft = buffer[12];
        input->mouseRight = buffer[13];


        //remote wheel
        input->remoteWheel = (buffer[17] << 8) | buffer[16];

    return true;
    }
}