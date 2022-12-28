#include <Arduino.h>
#include "buff_hid.h"


void HID_Packet::print_packet(){
	/*
		Display function for HID packets
		@param:
			None
		@return:
			None
	*/
	Serial.println("\n\t=====");
	for (int i = 0; i < HID_PACKET_SIZE_BYTES - 15; i += 16){
		Serial.printf("\t[%d]\t\t%X\t%X\t%X\t%X\t%X\t%X\t%X\t%X", i,
						data[i], data[i+1], data[i+2], data[i+3],
						data[i+4], data[i+5], data[i+6], data[i+7]);
		Serial.printf("\t%d\t%X\t%X\t%X\t%X\t%X\t%X\t%X\n", 
						data[i+8], data[i+9], data[i+10], data[i+11],
						data[i+12], data[i+13], data[i+14], data[i+15]);
	}
}

void HID_Packet::clear(){
	/*
		Clear the HID packet (set = 0)
		@param:
			None
		@return:
			None
	*/
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++){
		data[i] = 0;
	}
}

int8_t HID_Packet::write(){
	/*
		Write the packets bytes to usb
		@param:
			None
		@return:
			n: number of bytes written
	*/
	int8_t n = usb_rawhid_send(&data, 0);
	return n;
}

int8_t HID_Packet::read(){
	/*
		Read the usb bytes to the packet.
			check if available then reads.
		@param:
			None
		@return:
			n: number of bytes read
	*/
	int8_t n = 0;

	if (usb_rawhid_available()) {
		n = usb_rawhid_recv(&data, 0);
	}
	return n;
}

byte HID_Packet::get(int idx){
	/*
		Get the byte at packet[idx]
		@param:
			None
		@return:
			byte: requested byte
	*/
	return data[idx];
}

void HID_Packet::put(int idx, byte value){
	/*
		Set the byte at packet[idx]
		@param:
			None
		@return:
			None
	*/
	data[idx] = value;
}

void HID_Packet::rgets(byte* out_data, int offset, int bytes){
	/*
		Recursively Get the bytes at packet[offset:offset + bytes]
		@param:
			None
		@return:
			None
	*/
	out_data[bytes - 1] = data[offset + (bytes - 1)];

	if (bytes > 1) {
		rgets(out_data, offset, bytes - 1);
	}
}

void HID_Packet::rputs(byte* in_data, int offset, int bytes){
	/*
		Recursively Set the bytes at packet[offset:offset + bytes]
		@param:
			None
		@return:
			None
	*/
	data[offset + (bytes - 1)] = in_data[bytes - 1];

	if (bytes > 1) {
		rputs(in_data, offset, bytes - 1);
	}
}








