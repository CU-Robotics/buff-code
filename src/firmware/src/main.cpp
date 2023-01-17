#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us / 1E6;


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
	timer_set(0);

	// handle any hid input output
	device_manager.read_sensors();
	device_manager.step_controllers(cycle_time_s);
	device_manager.hid_input_switch();
	device_manager.push_can();

	timer_wait_us(0, cycle_time_us);
}
