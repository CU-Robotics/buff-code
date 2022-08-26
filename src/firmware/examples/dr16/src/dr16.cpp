#include "dr16.h"

DR16::DR16() {

}

void DR16::init(int idx) {
    id = idx;
    d_t = micros();
    Serial.print("New DR16 "); Serial.println(SERIAL_8E1_RXINV_TXINV); 

    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
}

void DR16::read() {
    
    // if (micros() - d_t < 5){
    //     return;
    // }

    if (Serial5.available() % 18 == 0)  //if the number of bytes is not divisible by 18 then there is a mangled frame and all data should be thrown out
    {
        byte buf[18];

        if(Serial5.available() >= 18) {
            Serial5.readBytes(buf, 18);
            Serial.print("DR16 reading ");

            for (int i = 0; i < 18; i++){
                Serial.print(buf[i]);
                Serial.print(" ");
            }
            Serial.println("");
        }
        d_t = micros();
    }
    Serial5.clear();             
}