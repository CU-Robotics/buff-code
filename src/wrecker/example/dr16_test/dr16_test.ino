#ifndef DR16_H
#include "dr16.h"
#endif

#ifndef STRUCTS_H
#include "structs.h"
#endif

dr16 reciever;
RobotInput inStruct;

void setup() {
    reciever.init(&inStruct);
    Serial.begin(115200);
    Serial.println("dr16 test");

    //debugging
    while (Serial5.available())
    {
        Serial5.read();
    }
    
}

void loop() {
    reciever.update();
    // Serial.print(inStruct.CH0);
    // Serial.print(", ");
    // Serial.println(inStruct.CH0, BIN);

    // Serial.print(inStruct.CH1);
    // Serial.print(", ");
    // Serial.println(inStruct.CH1, BIN);
    
    // Serial.print(inStruct.CH2);
    // Serial.print(", ");
    // Serial.println(inStruct.CH2, BIN);
    
    // Serial.println(inStruct.CH3);
    // Serial.println(inStruct.CH3, BIN);
    // Serial.print(inStruct.w);
    // Serial.print(", ");
    // Serial.print(inStruct.a);
    // Serial.print(", ");
    // Serial.print(inStruct.s);
    // Serial.print(", ");
    // Serial.print(inStruct.d);
    // Serial.print(", ");
    // Serial.print(inStruct.q);
    // Serial.print(", ");
    // Serial.print(inStruct.e);
    // Serial.print(", ");
    // Serial.print(inStruct.shift);
    // Serial.print(", ");
    // Serial.print(inStruct.ctrl);
    // Serial.print(", ");

    // Serial.print(inStruct.f);
    // Serial.println(String(0b1111111111111111, DEC));
    // Serial.println();
    // Serial.println(inStruct.mouseX);
    // Serial.print(', ');
    // Serial.println(inStruct.mouseY);
    // Serial.print(', ');
    // Serial.println(inStruct.mouseZ);
    // Serial.println(inStruct.remoteWheel);
    // Serial.println();
    // delayMicroseconds(5000);
}