#include <Arduino.h>

#include "drivers/dr16.h"
#include "drivers/mpu6050.h"
#include "drivers/rmmotor.h"
#include "drivers/buff_can.h"
#include "algorithms/Buffers.h"

#ifndef BUFFHID_H
#define BUFFHID_H

struct HID_Device {
  HIDBuffer input;
  HIDBuffer output;

  MPU6050 imu;
  DR16 receiver;
  BuffCan can;
};

void init_HID(HID_Device*);
int8_t send_HID(HID_Device*); // writes HID packet
int8_t read_HID(HID_Device*); // read HID packet

#endif