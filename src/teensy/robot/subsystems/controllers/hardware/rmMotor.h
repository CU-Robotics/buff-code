#ifndef _FLEXCAN_T4_H_
#include "../../../libraries/FlexCAN_T4-master/FlexCAN_T4.h"
#endif
#ifndef CONSTANTS_H
#include "../../../constants.h"
#endif

class rmMotor {
  public:
    rmMotor(short tempID, CAN_message_t &msg);
  private:
    short id;
    CAN_message_t &sendMsg;
    int byteNum;
};
