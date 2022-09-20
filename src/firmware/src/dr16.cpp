#include "dr16.h"

union {
    float num;
    byte bytes[4];
  } F2BUnion;

void print_receiver_input(byte* buffer){
    for (int i = 0; i < 18; i++)
    {
        for (int j = 7; j >= 0; j--)
        {
            Serial.print(bitRead(buffer[i], j));
        }
        Serial.print(" ");
    }
    Serial.println();
}


DR16::DR16()
{
    d_t = micros();
    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
}

void DR16::read(byte *buffer, int offset)
{

    if (Serial5.available() < 18)
    {
        return;
    }
    else if (Serial5.available() % 18 != 0)
    {
        Serial5.clear();
        return;
    }

    if (Serial5.available() >= 18)
    {
        byte tmp[18];
        Serial5.readBytes(tmp, 18);
        d_t = micros();

        buffer[offset] = 2;

        // debugging
        // print_receiver_input(tmp);

        // need to convert to normalized values
        uint16_t r_stick_x = ((tmp[1] & 0b00000111) << 8) | tmp[0];
        uint16_t r_stick_y = ((tmp[2] & 0b11111100) << 5) | ((tmp[1] & 0b11111000) >> 3);
        uint16_t l_stick_x = (((tmp[4] & 0b00000001) << 10) | (tmp[3] << 2)) | (tmp[2] & 0b00000011);
        uint16_t l_stick_y = ((tmp[5] & 0b00001111) << 7) | (tmp[4] & 0b11111110);

        byte s1 = ((tmp[5] & 0b00110000) >> 4);
        byte s2 = ((tmp[5] & 0b11000000) >> 6); // Set it up like this.

        F2BUnion.num = float(r_stick_x) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+1+i] = F2BUnion.bytes[3-i];
        }

        F2BUnion.num = float(r_stick_y) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+5+i] = F2BUnion.bytes[3-i];
        }

        F2BUnion.num = float(l_stick_x) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+9+i] = F2BUnion.bytes[3-i];
        }

        F2BUnion.num = float(l_stick_y) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+13+i] = F2BUnion.bytes[3-i];
        }

        F2BUnion.num = s1;
        for(int i = 0; i < 4; i++) {
            buffer[offset+17+i] = F2BUnion.bytes[3-i];
        }

        F2BUnion.num = s2;
        for(int i = 0; i < 4; i++) {
            buffer[offset+21+i] = F2BUnion.bytes[3-i];
        }

        return;
    }
}