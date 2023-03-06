#include <Arduino.h>

#ifndef ANALOG_PINS_H
#define ANALOG_PINS_H

#define MAX_PINS 8


struct Teensy_IO {
	Teensy_IO();
	bool setup_pin(int, int);
	void write_pin(int, int);
	int read_pin(int);

	int num_pins;
	int pins[MAX_PINS];
};

#endif