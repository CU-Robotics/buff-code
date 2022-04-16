#include "swerveChassis.h"

swerveChassis::swerveChassis() {
    //init can bus
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANbus;
    CAN_message_t sendMsg1;
    CAN_message_t sendMsg2;
    //init motors
    c610Enc FLMotor = c610Enc(1, &sendMsg, 6);
    c610Enc FRMotor = c610Enc(2, &sendMsg, 7);
    c610Enc BLMotor = c610Enc(3, &sendMsg, 8);
    c610Enc BRMotor = c610Enc(4, &sendMsg, 9);

    
    //init swerveModules
}