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

    Serial5.readBytes(buffer, 18);
    d_t = micros();
    return true;
}