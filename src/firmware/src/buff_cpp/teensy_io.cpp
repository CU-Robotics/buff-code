#include "teensy_io.h"


Teensy_IO::Teensy_IO() {
	num_pins = 0;
}
	
bool Teensy_IO::setup_pin(int pin, int mode) {
	if (num_pins == MAX_PINS) {
		return false;
	}

	pinMode(pin, mode);
	analogWrite(pin, 0);
	pins[num_pins] = pin;
	num_pins ++;
	return true;
}

void Teensy_IO::write_pin(int pin, int value) {
	if (0 < pin && pin < num_pins)
		analogWrite(pins[pin], value);
}

int Teensy_IO::read_pin(int pin) {
	if (0 < pin && pin < num_pins)
		return analogRead(pin);
	else
		return -1;
}
