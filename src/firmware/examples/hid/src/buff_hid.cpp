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
		  Clear the HID packet (set all indices = 0)
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
		  Write the packet's bytes to usb
		@param:
			None
		@return:
			n: number of bytes written
	*/
	return usb_rawhid_send(&data, 0);
}

int8_t HID_Packet::read(){
	/*
		  Read the usb bytes to the packet.
		@param:
			None
		@return:
			n: number of bytes read
	*/
	return usb_rawhid_recv(&data, 0);
}

byte HID_Packet::get(int idx){
	/*
		  Get the byte at packet[idx]
		@param:
			idx: index of byte to read
		@return:
			byte: requested byte
	*/
	return data[idx];
}

void HID_Packet::put(int idx, byte value){
	/*
		  Set the byte at packet[idx]
		@param:
			idx: index of the byte to set
			value: the byte to write
		@return:
			None
	*/
	data[idx] = value;
}

int32_t HID_Packet::get_int32(int idx){
	/*
		  Get the int32_t at packet[idx]
		@param:
			idx: index of the value to read
		@return:
			int32_t: requested data
	*/
	return  (int32_t(data[idx]) << 24) | 
				(int32_t(data[idx+1]) << 16) | 
				(int32_t(data[idx+2]) << 8) | 
				 int32_t(data[idx+3]);
}

void HID_Packet::put_int32(int idx, int32_t value){
	/*
		  Set the int32_t at packet[idx]
		@param:
			idx: index of the data to write to
			value: the value to write
		@return:
			None
	*/
	data[idx]   = byte(value >> 24);
	data[idx+1] = byte(value >> 16);
	data[idx+2] = byte(value >> 8);
	data[idx+3] = byte(value);
}

float HID_Packet::get_float(int idx){
	/*
		  Get the float at packet[idx], floats are converted into
		byte* arrays for compressed storage, atm we use unions
		to do this (should be safe on this arm processor, but 
		definitely warrants inspection). No-Endian
		@param:
			idx: the index to start reading the float from
		@return:
			float: requested data
	*/
	FLOATBYTE_t fb_union;
	fb_union.bytes[0] = data[idx];
	fb_union.bytes[1] = data[idx+1];
	fb_union.bytes[2] = data[idx+2];
	fb_union.bytes[3] = data[idx+3];

	return fb_union.number;
}

void HID_Packet::put_float(int idx, float value){
	/*
		  Set float at packet[idx], use union to
		turn float to byte* and insert at index 0.
		(NE)
		@param:
			idx: index to insert at
			value: the float value to insert
		@return:
			None
	*/
	FLOATBYTE_t fb_union;
	fb_union.number = value;
	data[idx]   = fb_union.bytes[0];
	data[idx+1] = fb_union.bytes[1];
	data[idx+2] = fb_union.bytes[2];
	data[idx+3] = fb_union.bytes[3];
}

char* HID_Packet::get_chars(int idx, int n){
	/*
		  Get the char* at packet[idx]
		@param:
			idx: index of the chars
			n: number of chars to read
		@return:
			char*: requested data
	*/

	char* s;
	for (int i = 0; i < n; i++) {
		s += data[idx + i];
	}
	return s;
}

void HID_Packet::put_chars(int idx, char* value){
	/*
		Set the int32_t at packet[idx]
		@param:
			idx: index to begin inserting
			value: list of chars to insert
		@return:
			None
	*/
	for (size_t i = 0; i < strlen(value); i++) {
		data[idx + i] = value[i];
	}
}

void HID_Packet::rgets(byte* out_data, int offset, int bytes){
	/*
		  Recursively Get the bytes at packet[offset:offset + bytes]
		@param:
			out_data: a byte buffer to fill with data
			offset: the index to begin reading from
			bytes: the number of bytes to get
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
			in_data: a byte buffer to copy data from
			offset: the index to begin writing at
			bytes: the number of bytes to write
		@return:
			None
	*/
	data[offset + (bytes - 1)] = in_data[bytes - 1];

	if (bytes > 1) {
		rputs(in_data, offset, bytes - 1);
	}
}

void HID_Packet::check_hid_input(HID_Packet* outgoing_report){
	switch (this.read()) {
		case 64:
			blink();

			switch (this.get(0)) {
				case 0:
					outgoing_report.put(0, 0);
					// Fill init device handlers
					break;

				case 1:
					outgoing_report.put(0, 1);
					// Fill with can bus 1 info
					break;

				case 2:
					outgoing_report.put(0, 2);
					// Fill with can bus 2 info
					break;

				case 3:
					outgoing_report.put(0, 3);
					// Fill with DR16/IMU info
					for (int i = 0; i < 9; i++){
						outgoing_report.put_float((i * 4) + 1, 3.1415926);
					}
					break;

				default:
					// On intial connection the HID packet is
					// set to init_mode
					outgoing_report.put(0, 255);
					break;
			}

			// clock cycles since loop timer
			delta_us = DURATION_US(loop_timer, ARM_DWT_CYCCNT);

			// set the duration of the loop as the last 4 values
			// in the packet and write the packet
			outgoing_report.put_int32(60, delta_us);
			outgoing_report.write();
			break;

		default:
			delta_us = DURATION_US(loop_timer, ARM_DWT_CYCCNT);
			break;
	}
}








