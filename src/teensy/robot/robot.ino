#include "./libraries/FlexCAN_T4-master/FlexCAN_T4.h"

#include "./subsystems/controllers/hardware/rmMotor.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

c620 myMotor(1, &msg);

void setup() {
  
}

void loop() {
  
}
