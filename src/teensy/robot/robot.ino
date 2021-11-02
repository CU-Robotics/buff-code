//#include <FlexCAN_T4.h>
#include "./libraries/FlexCAN_T4-master/FlexCAN_T4.h"

#include "./subsystems/controllers/hardware/c620.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

c620 myMotor(1, &msg);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);
}

void loop() {
  
}
