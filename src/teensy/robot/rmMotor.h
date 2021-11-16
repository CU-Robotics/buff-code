#ifndef CONSTANTS_H
#include "constants.h"
#endif

class rmMotor {
  public:
    // rmMotor(short tempID, CAN_message_t* msg);
    short getTorque();
    short getRpm();
    short getAngle();
    byte getTemp();
  private:    //may need to be protected instaed of private if subclasses need to access values
    short id;
    int byteNum = -1;
    short torque = -1;
    short rpm = -1;
    short angle = -1;
    byte temp = -1;
};
