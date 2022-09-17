#include "dr16.h"

DR16::DR16()
{
    d_t = micros();
    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
}

bool DR16::read(byte *buffer)
{

    if (Serial5.available() < 18)
    {
        return false;
    }
    else if (Serial5.available() % 18 != 0)
    {
        Serial5.clear();
        return false;
    }

    if (Serial5.available() >= 18)
    {
        byte tmp[18];
        Serial5.readBytes(tmp, 18);
        d_t = micros();

        numBytes = 0;

        // debugging
        //  for (int i = 0; i < 18; i++)
        //  {
        //      for (int j = 7; j >= 0; j--)
        //      {
        //          Serial.print(bitRead(buf[i], j));
        //      }
        //      Serial.print(" ");
        //  }
        //  Serial.println();

        buffer[34] = ((tmp[1] & 0b00000111) << 8) | tmp[0];
        buffer[35] = ((tmp[2] & 0b11111100) << 5) | ((tmp[1] & 0b11111000) >> 3);
        buffer[37] = (((tmp[4] & 0b00000001) << 10) | (tmp[3] << 2)) | (buf[2] & 0b00000011);
        buffer[38] = ((tmp[5] & 0b00001111) << 7) | (tmp[4] & 0b11111110);

        buffer[36] = ((tmp[5] & 0b00110000) >> 4);
        buffer[39] = ((tmp[5] & 0b11000000) >> 6); // Set it up like this.

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

        return true;
    }
}