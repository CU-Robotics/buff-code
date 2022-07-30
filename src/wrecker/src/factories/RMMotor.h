#include <FlexCAN_T4.h>


/*
  struct motor_init_data{
    short tempID;
    short CanBusNum;
  };
*/

#ifndef RMMOTOR_H
#define RMMOTOR_H

// Create a struct containing the data
// required to spin-up each of the three
// motors (gm6020, c620, c610). Assume
// that this is filled by the HID/Serial
// driver
struct rmMotor_init_data{
  bool unread_marker = false;
};

// Create a struct that contains all the
// non-changing data (inputs and config)
struct rmMotor_config{
  short byteNum = -1;
  short MAX_VALUE = -1;

  uint8_t id = -1;
  uint8_t canBusID = -1;
  CAN_message_t *sendMsgPtr;
};

// Create a struct that contains all the
// changing data (sensor readings, generated values)
struct rmMotor_state{

  float p_ref = -1;
  float v_ref = -1;
  float t_ref = -1;

  float p_act = -1;
  float v_act = -1;
  float t_act = -1;
};

struct rmMotor{
  bool is_gm6020;
  rmMotor_config config;
  rmMotor_state state;
};

struct rmMotor_LUT{
  // setup CAN and motor-lookup
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
  FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
  FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

  CAN_message_t canRecieveMessages[3][11];
  CAN_message_t gm6020Messages[3][2];
  CAN_message_t c6x0Messages[3][2];

  // there is definitely a better way to store/ access motor data (tree/graph?)
  rmMotor* LUT[30];

  int n_items = 0;

};

// create and store a new motor (config,state), somewhat analgous to init
// Expect that the input to this comes directly from serial (or close too).
// motor_init_data is not motor_config or motor_state. This will be found
// by integrating the motors inits and using the struct to handle incompatibilities.
rmMotor* RMMotor_Factory(rmMotor_LUT*, uint8_t, uint8_t, uint8_t, bool);

// Maybe this should be a more general init_func
void initCAN(rmMotor_LUT*);

// update a single motors IO
void updateRMMotor(rmMotor_LUT*, rmMotor*);

// update the IO signals to all the motors in the table
// I could see reasons to do this differently
void updateIO(rmMotor_LUT*);


void writeCX20_CAN(rmMotor_LUT*);
void writeGM6020_CAN(rmMotor_LUT*);

// Write the can packets
void writeCAN(rmMotor_LUT*);

void setPower(rmMotor_LUT*, rmMotor*, short);
void setRPM(rmMotor_LUT*, rmMotor*, short);
void setTorque(rmMotor_LUT*, rmMotor*, short);
void setAngle(rmMotor_LUT*, rmMotor*, short);


#endif
// Stay inside the #if