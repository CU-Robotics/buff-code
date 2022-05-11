#include <Arduino.h>
#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#include "drivers/ref_sys.h"

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

ref_sys referee_system;

void setup() {
    Serial.begin(115200);
    Serial.println("Referee system test");
}

void loop() {

Serial.print("start: ");
Serial.println(micros());
referee_system.read_serial();
Serial.print("end: ");
Serial.println(micros());
Serial.println();
delay(10);


}