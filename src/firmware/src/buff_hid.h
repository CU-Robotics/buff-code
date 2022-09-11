#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef BUFFHID_H
#define BUFFHID_H


struct HID_Packet {
  public:
    byte data[64];
};

void clear(HID_Packet*);
int8_t write_HID(HID_Packet*);
int8_t read_HID(HID_Packet*);

void set_can_id(HID_Packet*, uint8_t);
int get_can_id(HID_Packet* hid);

void set_sensor_id(HID_Packet*, uint8_t);
int get_sensor_id(HID_Packet*);

void get_device_input(HID_Packet*, byte*, int, int);
void get_can_input(HID_Packet*, CAN_message_t*);
void set_device_output(HID_Packet*, byte*, int, int);

#endif