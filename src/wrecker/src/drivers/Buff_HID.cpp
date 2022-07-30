#include <Arduino.h>

#include "Buff_HID.h"

int send_HID(HID_Device* hid){
  for (int i = 0; i < 64; i++){
    hid->buffer[i] = byte(i);
  }

  return RawHID.send(hid->buffer, 100);
}

int read_HID(HID_Device* hid){
	int n = RawHID.recv(hid->buffer, 0);
}