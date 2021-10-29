#ifndef FLEXCAN_H
#include <FlexCAN_T4.h>
#endif

class rmMotor {
  public:
  rmMotor(short id, CAN_message_t msg);
  void set(short power);
  private:
};
