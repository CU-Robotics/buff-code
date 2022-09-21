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

        uint8_t numBytes = 0;
        float r_stick_x_scale = (1684.0 - 364.0) / 2.0;
        float r_stick_y_scale = (1684.0 - 268.0) / 2.0;
        float l_stick_x_scale = (1684.0 - 364.0) / 2.0;
        float l_stick_y_scale = (1704.0 - 476.0) / 2.0;

        // debugging
        // print_receiver_input(tmp);

        // need to convert to normalized values
        uint16_t r_stick_x = ((tmp[1] & 0b00000111) << 8) | tmp[0];
        uint16_t r_stick_y = ((tmp[2] & 0b11111100) << 5) | ((tmp[1] & 0b11111000) >> 3);
        uint16_t l_stick_x = (((tmp[4] & 0b00000001) << 10) | (tmp[3] << 2)) | (tmp[2] & 0b00000011);
        uint16_t l_stick_y = ((tmp[5] & 0b00001111) << 7) | (tmp[4] & 0b11111110);

        // Serial.print("r_stick_x ");
        // Serial.print(r_stick_x);

        byte s1 = ((tmp[5] & 0b00110000) >> 4);
        byte s2 = ((tmp[5] & 0b11000000) >> 6); // Set it up like this.

        F2BUnion.num = float(r_stick_x) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+1+i] = F2BUnion.bytes[i];
        }

        F2BUnion.num = float(r_stick_y) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+5+i] = F2BUnion.bytes[i];
        }

        F2BUnion.num = float(l_stick_x) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+9+i] = F2BUnion.bytes[i];
        }

        F2BUnion.num = float(l_stick_y) - 1024.0;
        for(int i = 0; i < 4; i++) {
            buffer[offset+13+i] = F2BUnion.bytes[i];
        }

        buffer[offset+17] = s1;
        buffer[offset+18] = s2;      

        // buffer[offset+1] = int8_t((float(r_stick_x) - 1024.0));
        // buffer[offset+2] = int8_t((float(r_stick_y) - 1024.0) / r_stick_y_scale);
        // buffer[offset+3] = int8_t((float(l_stick_x) - 1024.0) / l_stick_x_scale);
        // buffer[offset+4] = int8_t((float(l_stick_y) - 1024.0) / l_stick_y_scale);
        // buffer[offset+5] = s1;
        // buffer[offset+6] = s2;

        // Serial.print(" scaled ");
        // Serial.println(buffer[offset+1]);

        // Serial.println(input->S1, BIN);

        // first byte of keyboard
        // input->w = tmp[14] & 0b00000001;
        // input->s = tmp[14] & 0b00000010;
        // input->a = tmp[14] & 0b00000100;
        // input->d = tmp[14] & 0b00001000;
        // input->shift = tmp[14] & 0b00010000;
        // input->ctrl = tmp[14] & 0b00100000;
        // input->q = tmp[14] & 0b01000000;
        // input->e = tmp[14] & 0b10000000;

        // // second byte of keyboard
        // input->r = tmp[15] & 0b00000001;
        // input->f = tmp[15] & 0b00000010;
        // input->g = tmp[15] & 0b00000100;
        // input->z = tmp[15] & 0b00001000;
        // input->x = tmp[15] & 0b00010000;
        // input->c = tmp[15] & 0b00100000;
        // input->v = tmp[15] & 0b01000000;
        // input->b = tmp[15] & 0b10000000;

        // // mouse
        // input->mouseX = (int16_t)(tmp[6] | (tmp[7] << 8));
        // input->mouseY = (int16_t)(tmp[8] | (tmp[9] << 8));
        // input->mouseZ = (int16_t)(tmp[10] | (tmp[11] << 8));
        // input->mouseLeft = tmp[12];
        // input->mouseRight = tmp[13];

        // // remote wheel
        // input->remoteWheel = (tmp[17] << 8) | tmp[16];

        return;
    }
}