#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us / 1E6;


Device_Manager device_manager;						// all firmware pipelines are implemented in this object.
													// Device Manager provides a single function call for each of the pipelines
													// with unit tests we can analyze the execution time and complexity of each
													// pipeline. Then organize them into the master loop accordingly

// Runs once
void setup() {
	Serial.begin(1000000);							// the serial monitor is actually always active (for debug use Serial.println & tycmd)

	if (Serial) {
		Serial.println("-- TEENSY SERIAL START --");
	}
}

// Master loop
// Lets ditch the arduino framework
// it feels immature, or maybe thats dumb
void loop() {										// Basically a schudeling algorithm
	timer_set(0);

	// handle any hid input output
	device_manager.read_sensors();					// read a single sensor each call (increments the sensor id automatically)
	device_manager.step_controllers(cycle_time_s);		// given the current inputs and feedback compute a control
	device_manager.hid_input_switch();				// check for an input packet (data request/control input) handle accordingly
	device_manager.push_can();						// push data on and off the can bus

	timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_us
}
