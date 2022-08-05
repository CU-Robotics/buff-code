#include <Arduino.h>

#include "drivers/dr16.h"
#include "drivers/mpu6050.h"
#include "drivers/rmmotor.h"
#include "algorithms/Buffers.h"

#ifndef BUFFHID_H
#define BUFFHID_H

struct HID_Device {
  HIDBuffer input;
  HIDBuffer output;

  MPU6050 imu;
  DR16 receiver;
};

void init_HID(HID_Device*);
int send_HID(HID_Device*, Motor_LUT*); // writes HID packet
int read_HID(HID_Device*, Motor_LUT*); // read HID packet

#endif