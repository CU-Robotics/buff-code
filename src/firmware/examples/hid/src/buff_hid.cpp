#include <Arduino.h>

#include "buff_hid.h"


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

void set_device_output(HID_Packet* hid, byte* data, int offset, int bytes){
	hid->data[offset + (bytes - 1)] = data[bytes - 1];

	if (bytes > 1) {
		set_device_output(hid, data, offset, bytes - 1);
	}
}

void get_device_input(HID_Packet* hid, byte* data, int offset, int bytes){
	data[bytes - 1] = hid->data[offset + (bytes - 1)];

	if (bytes > 1) {
		get_device_input(hid, data, offset, bytes - 1);
	}
}

void set_can_output(HID_Packet* hid, CAN_message_t* msg){
	byte id = ((msg->id - 0x200) % 4) - 1;
	int offset = (((msg->id - 0x200) / 4) * 9) + 1;

	set_device_output(hid, &id, offset, 1);
	set_device_output(hid, msg->buf, offset + 1, 8);
}

int get_can_input(HID_Packet* hid, CAN_message_t* msgs){
	for (int i = 0; i < 3; i++){
		get_device_input(hid, msgs[i].buf, (i * 8) + 1, 8);
	}
	return get_can_id(hid);
}

void parse_can1_output(HID_Packet* hid, CAN_message_t* msg){
	set_can_id(hid, 1);
	set_can_output(hid, msg);
}

void parse_can2_output(HID_Packet* hid, CAN_message_t* msg){
	set_can_id(hid, 2);
	set_can_output(hid, msg);
}

void parse_dr16_output(HID_Packet* hid, byte* buffer){
	set_sensor_id(hid, 1);
	set_device_output(hid, buffer, 31, 18);
}

void parse_mpu6050_output(HID_Packet* hid, byte* buffer){
	set_sensor_id(hid, 2);
	set_device_output(hid, buffer, 31, 24);
}