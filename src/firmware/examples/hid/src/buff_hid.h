#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef BUFFHID_H
#define BUFFHID_H

#define HID_PACKET_SIZE_BYTES 64

struct HID_Packet {
	byte data[HID_PACKET_SIZE_BYTES];

	void clear();
	void print_packet();

	int8_t read();
	int8_t write();

	// Getters/Setters
	byte get(int);
	void put(int, byte);

	void rgets(byte*, int, int);
	void rputs(byte*, int, int);
};

#endif