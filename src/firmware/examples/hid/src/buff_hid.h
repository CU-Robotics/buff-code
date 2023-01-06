#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef BUFFHID_H
#define BUFFHID_H

#define HID_PACKET_SIZE_BYTES 64

typedef union
{
	float number;
	uint8_t bytes[4];
} FLOATBYTE_t;

/*
	A Packet struct to clean up the inserts and reads.
	The goal is to have an hid.fill_report_xxx(data),
	that will put the data in our HID report structure and
	send it to serial. Similarly hid.read() should decompose
	the recieved report into smaller data.
	
	the main priority is putting data in and out of the packet.
	There is some discussion to be had on whose job it is to
	determine indexing, for now it will be the developers responsibility.

	Get/Set requirements:
		IMU:  			float[9]
		
		DR16: 			byte[18]

		CAN:  			int16[i][3] (feedback)
			  			int16[i]    (command)

		Teensy clock:	int32

	HID_Packet also has quick functions to
	read and write its data.
*/

struct HID_Packet {
	byte data[HID_PACKET_SIZE_BYTES];

	// for debug, print the packet with serial.print
	void print_packet();
	// clear all data in the packet
	void clear();

	// fill data with the available HID packet, (wait if none?)
	int8_t read();
	// send data as HID packet
	int8_t write();

	// Getters/Setters
	byte get(int);
	void put(int, byte);

	int32_t get_int32(int);
	void put_int32(int, int32_t);

	float get_float(int);
	void put_float(int, float);

	char* get_chars(int, int);
	void put_chars(int, char*);

	void rgets(byte*, int, int);
	void rputs(byte*, int, int);

	// state machine 
	void check_hid_input(HID_Packet*);
};

#endif