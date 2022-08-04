#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "RMMotor.h"

rmMotor* RMMotor_Factory(rmMotor_LUT* motor_lut, uint8_t id, uint8_t canBusNum, bool is_gm6020){
  // Go in each motors init function try to replicate what happens here

  rmMotor* motor = new rmMotor;

  motor->config.id = id;
  motor->config.byteNum = id - 1;
  motor->config.canBusID = canBusNum;


  if(motor->config.byteNum > 3) {
    motor->config.byteNum -= 4;
    if(is_gm6020){
      motor->config.sendMsgPtr = &motor_lut->gm6020Messages[canBusNum-1][0];
      motor->config.sendMsgPtr->id = 0x2FF;   //ID for all gm6020s 5-7
    }
    else {
      motor->config.sendMsgPtr = &motor_lut->c6x0Messages[motor->config.canBusID-1][0];
      motor->config.sendMsgPtr->id = 0x1FF;   //ID for all c620s 4-7
    }
  } 
  else {
    if(is_gm6020){
      motor->config.sendMsgPtr = &motor_lut->gm6020Messages[canBusNum-1][1];
      motor->config.sendMsgPtr->id = 0x1FF;   //ID for all gm6020s 0-3
    }
    else {
      motor->config.sendMsgPtr = &motor_lut->c6x0Messages[motor->config.canBusID-1][1];
      motor->config.sendMsgPtr->id = 0x200;   //ID for all c620s 0-3
    }
  }

  return motor;
}

// Maybe this should be a more general init_func
void initCAN(rmMotor_LUT* motor_lut){    

  motor_lut->can1.begin();
  motor_lut->can2.begin();
  motor_lut->can3.begin();

  motor_lut->can1.setBaudRate(1000000);
  motor_lut->can2.setBaudRate(1000000);
  motor_lut->can3.setBaudRate(1000000);
}

void updateRMMotor(rmMotor_LUT* lut, rmMotor* motor){
  // use the update functions from the motor classes and replicate them here
  CAN_message_t *recMsg = &(lut->canRecieveMessages[motor->config.canBusID - 1][motor->config.id + 3]);
  
  uint16_t temp;
  temp = recMsg->buf[0];
  temp = temp << 8;
  temp = temp | recMsg->buf[1];
  motor->state.p_ref = map(temp, 0, 8191, 0, 36000) / 100.0;

  temp = recMsg->buf[2];
  temp = temp << 8;
  motor->state.v_ref = temp | recMsg->buf[3];

  temp = recMsg->buf[4];
  temp = temp << 8;
  motor->state.t_ref = temp | recMsg->buf[5];

  temp = recMsg->buf[6];
}

void updateIO(rmMotor_LUT* motor_lut){
  int i = 0;
  while (i < motor_lut->n_items){
    updateRMMotor(motor_lut, motor_lut->LUT[i]);
    i ++;
  }
}

void writeCX20_CAN(rmMotor_LUT* motor_lut){
  motor_lut->can1.write(motor_lut->c6x0Messages[0][0]);
  motor_lut->can1.write(motor_lut->c6x0Messages[0][1]);
  motor_lut->can2.write(motor_lut->c6x0Messages[1][0]);
  motor_lut->can2.write(motor_lut->c6x0Messages[1][1]);
  motor_lut->can3.write(motor_lut->c6x0Messages[2][0]);
  motor_lut->can3.write(motor_lut->c6x0Messages[2][1]);
}

void writeGM6020_CAN(rmMotor_LUT* motor_lut){
  motor_lut->can1.write(motor_lut->gm6020Messages[0][0]);
  motor_lut->can1.write(motor_lut->gm6020Messages[0][1]);
  motor_lut->can2.write(motor_lut->gm6020Messages[1][0]);
  motor_lut->can2.write(motor_lut->gm6020Messages[1][1]);
  // can3.write(c6x0Messages[2][0]);
  // can3.write(c6x0Messages[2][1]);
}

void writeCAN(rmMotor_LUT* motor_lut){
  writeCX20_CAN(motor_lut);
  writeGM6020_CAN(motor_lut);
}

void setPower(rmMotor_LUT* motor_lut, rmMotor* motor, short power){

}

void setAngle(rmMotor_LUT* motor_lut, rmMotor* motor, short angle){
  motor->state.p_ref = angle;
  updateRMMotor(motor_lut, motor);
}

void setRPM(rmMotor_LUT* motor_lut, rmMotor* motor, short rpm){
  motor->state.v_ref = rpm;
  updateRMMotor(motor_lut, motor);
}

void setTorque(rmMotor_LUT* motor_lut, rmMotor* motor, short torque){
  motor->state.t_ref = torque;
  updateRMMotor(motor_lut, motor);
}