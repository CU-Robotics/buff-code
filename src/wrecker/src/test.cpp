#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <drivers/c620.h>

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
CAN_message_t canRecieveMessages[3][11];
CAN_message_t tempMessage;
int CANTimer = 0;

c620CAN my_motor;

uint8_t motor_id = 7, temp_can_num = 1; 
elapsedMillis num_millis_elapsed = 0;

void setup(){

    can1.begin();
    can2.begin();

    can1.setBaudRate(1000000);
    can2.setBaudRate(1000000);

    my_motor.init(motor_id,temp_can_num);


}

void loop(){

    if(num_millis_elapsed>50)
    {
        num_millis_elapsed = 0;

        my_motor.setPower(.05);

        sendC6x0();
    }

}

