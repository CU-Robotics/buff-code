#include "dr16.h"

DR16::DR16() {
    
}

void DR16::init(int idx) {
    id = idx;
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);  // Need more info on this @Tyler
}

void DR16::read(HIDBuffer* buffer) {

    if (Serial5.available() % 18 != 0)  //if the number of bytes is not divisible by 18 then there is a mangled frame and all data should be thrown out
    {
        Serial5.flush();
        return;
    }
    
    if (id != -1){
        byte buf[18];

        if(Serial5.available() >= 18) {
            Serial5.readBytes(buf, 18);
            if (!buffer->check_of(23)){
                buffer->put('X');
                buffer->put('X');
                buffer->put(18);
                buffer->put(id);
                buffer->put(0);

                for (int i = 0; i < 18; i++){
                    buffer->put(buf[i]);
                }
            }
        }
            
    }
    
}