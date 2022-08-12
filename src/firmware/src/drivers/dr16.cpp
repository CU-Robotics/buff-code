#include "dr16.h"

DR16::DR16() {
    
}

void DR16::init(int idx) {
    id = idx;
    d_t = micros();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
    // Serial.print("New dr16 "); Serial.println(idx); 
}

void DR16::read(HIDBuffer* buffer) {
    
    if (id == -1 || micros() - d_t < 5){
        return;
    }

    if (Serial5.available() % 18 != 0)  //if the number of bytes is not divisible by 18 then there is a mangled frame and all data should be thrown out
    {
        Serial5.flush();
        return;
    }
    byte buf[18];

    if(Serial5.available() >= 18) {
        Serial5.readBytes(buf, 18);
        if (!buffer->check_of(23)){
            buffer->put('T');
            buffer->put('T');
            buffer->put(18);
            buffer->put(id);

            for (int i = 0; i < 18; i++){
                buffer->put(buf[i]);
            }
        }
    }
    d_t = micros();
                
}