#include "c620.h"

c620::rmMotor(short tempID, CAN_message_t &msg) {
    id = tempID;
    &sendMsg = msg;
}

c620::set(float power) {
    
}