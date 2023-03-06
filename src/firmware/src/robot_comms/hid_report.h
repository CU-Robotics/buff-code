#include <Arduino.h>

#ifndef BUFFHID_REPORT_H
#define BUFFHID_REPORT_H

#define HID_REPORT_SIZE_BYTES 64

typedef union
{
	float number;
	byte bytes[4];
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

	HID_REPORT also has quick functions to
	read and write its data.
*/

struct Hid_Report {
	byte data[HID_REPORT_SIZE_BYTES];

	// constructor
	Hid_Report();

	// for debug, print the packet with serial.print
	void print();
	// clear all data in the packet
	void clear();

	// Getters/Setters
	byte get(int);
	void put(int, byte);

	int32_t get_int32(int);
	void put_int32(int, int32_t);

	float get_float(int);
	void put_float(int, float);

	void get_chars(int, int, char*);
	void put_chars(int, char*);

	void rgets(byte*, int, int);
	void rputs(byte*, int, int);

	// fill data with the available HID packet, (wait if none?)
	int8_t read();
	// send data as HID packet
	int8_t write();

};

#endif