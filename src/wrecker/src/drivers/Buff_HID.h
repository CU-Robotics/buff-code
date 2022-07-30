#include <Arduino.h>

#ifndef BUFFHID_H
#define BUFFHID_H

struct HID_Device {
  byte buffer[64];
  IntervalTimer i_tmr;
};

//void init_HID(HID_Device*);
int send_HID(HID_Device*);
int read_HID(HID_Device*);

#endif