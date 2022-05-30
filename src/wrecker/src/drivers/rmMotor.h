#include <FlexCAN_T4.h>

#ifndef RMMOTOR_H
#define RMMOTOR_H

class rmMotor {
  public:
    short getTorque();
    short getRpm();
    short getAngle();
    byte getTemp();
    void updateMotor();
    // void updateMotor(CAN_message_t* recMsg);     //update motor info values directly from CAN message  !!!!!!needs implementation
    // void updateMotor(short newTorque, short newRpm, short newAngle, byte newTemp);  //update motor info values by passing in values (that have been extracted from CAN message, for example)
  private:    //may need to be protected instaed of private if subclasses need to access values
    
  protected:
    short torque = -1;
    short rpm = -1;
    short angle = -1;
    byte temp = -1;
    uint8_t id = -1;
    uint8_t canBusNum = 0;
    short MAX_VALUE;
    short byteNum = -1;
    CAN_message_t *sendMsgPtr;
};

#endif // RMMOTOR_H