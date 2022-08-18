#include "dr16.h"

DR16::DR16() {}

void DR16::init(int idx) {
    id = idx;
    d_t = micros();
    // Serial.print("New DR16 "); Serial.println(SERIAL_8E1_RXINV_TXINV); 

    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV); 
}

void DR16::read(HIDBuffer* buffer) {

    if (Serial5.available() % 18 == 0)
    {
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
    Serial5.clear();
                
}