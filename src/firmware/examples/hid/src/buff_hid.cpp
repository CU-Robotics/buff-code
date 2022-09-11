#include <Arduino.h>

#include "buff_hid.h"

void clear(HID_Packet* hid){
	for (int i = 0; i < 64; i++) {
		hid->data = 0;
	}
}

int8_t write_HID(HID_Packet* hid){
	int8_t n = usb_rawhid_send(&hid->data, 0);
	return n;
}

int8_t read_HID(HID_Packet* hid){
	// Serial.println("Readng");
	int8_t n = usb_rawhid_recv(&hid->data, 0);
	return n;
}

void set_can_id(HID_Packet* hid, uint8_t id){
	hid->data[0] = id;
}

int get_can_id(HID_Packet* hid){
	return hid->data[0];
}

void set_sensor_id(HID_Packet* hid, uint8_t id){
	hid->data[30] = id;
}

int get_sensor_id(HID_Packet* hid){
	return hid->data[30];
}

void get_device_input(HID_Packet* hid, byte* data, int offset, int bytes){
	data[bytes - 1] = hid->data[offset + (bytes - 1)];

	if (bytes > 1) {
		get_device_input(hid, data, offset, bytes - 1);
	}
}

void get_can_input(HID_Packet* hid, CAN_message_t* msgs){ 	
	for (int i = 0; i < 3; i++){
		get_device_input(hid, msgs[i].buf, (i * 8) + 1, 8);
	}
}

void set_device_output(HID_Packet* hid, byte* data, int offset, int bytes){
	hid->data[offset + (bytes - 1)] = data[bytes - 1];

	if (bytes > 1) {
		set_device_output(hid, data, offset, bytes - 1);
	}
}