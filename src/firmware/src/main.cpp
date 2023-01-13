#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

#define DEVICE_READ_RATE 1000
#define CYCLE_TIME_US 1000
#define CYCLE_TIME_MS CYCLE_TIME_US / 1000


Device_Manager device_manager;

// Runs once
void setup() {

	Serial.begin(1000000);

	if (Serial) {
		Serial.println("-- TEENSY SERIAL START --");
	}
}

// Runs continuously
void loop() {
	timer_set(9);

	// handle any hid input output
	device_manager.read_sensors();
	device_manager.push_can();
	device_manager.hid_input_switch();

	timer_wait_us(9, CYCLE_TIME_US);
}
