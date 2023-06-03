#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
#include "buff_cpp/device_manager.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us * 1E-6;

unsigned long prev_time;


Device_Manager device_manager;						// all firmware pipelines are implemented in this object.
													// Device Manager provides a single function call for each of the pipelines
													// with unit tests we can analyze the execution time and complexity of each
													// pipeline. Then organize them into the master loop accordingly

// Runs once
void setup() {
	Serial.begin(1000000);							// the serial monitor is actually always active (for debug use Serial.println & tycmd)

	if (Serial) {
		Serial.println("-- TEENSY SERIAL START --");
		Serial.println("-- new build... who dis? --");
	}
}

// Master loop
int main() {											// Basically a schudeling algorithm
	setup();
	prev_time = micros();

	while(1) {
		timer_set(0);

		// Calculate dt
		unsigned long curr_time = micros();
		float dt = (curr_time - prev_time) / 1000000.0;
		prev_time = curr_time;

		// handle any hid input output
		device_manager.read_sensors();					// read a single sensor each call (increments the sensor id automatically)		
		device_manager.step_controllers(dt);	// given the current inputs and feedback compute a control	
		device_manager.hid_input_switch(cycle_time_us);	// check for an input packet (data request/control input) handle accordingly
		device_manager.push_can();						// push data on and off the can bus

		// if (timer_info_ms(0) > cycle_time_ms) {
		// 	Serial.println("Teensy overcycled");
		// }
		timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_u
		// blink();										// helpful if you think the loop is crashing (light will pause)
	}
	
	return 0;
}
