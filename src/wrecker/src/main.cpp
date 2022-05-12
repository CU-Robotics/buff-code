#include <Arduino.h>
#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

CAN_message_t canRecieveMessages[3][11];  //used by motor objects to access recieved data
#include "state/state.h"
#include "drivers/c620.h"
#include "drivers/gm6020.h"

#include "drivers/ref_sys.h"

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

ref_sys referee_system;
S_RefSystem * temp_for_test;

void setup() {
    Serial.begin(115200);
    Serial.println("Referee system test");
    referee_system.init(temp_for_test);
}

void loop() {

Serial.print("start: ");
Serial.println(micros());
referee_system.read_serial();
Serial.print("end: ");
Serial.println(micros());
Serial.println();
delay(10);
Serial.println(temp_for_test -> comp_type);



}