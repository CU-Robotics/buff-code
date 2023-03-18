// #include "unity.h"
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
		Serial.println("-- new build... who dis? --");
	}
}

// Master loop
int main() {											// Basically a schudeling algorithm
	setup();

	while(1) {
		timer_set(0);

		// handle any hid input output
		device_manager.read_sensors();					// read a single sensor each call (increments the sensor id automatically)
		device_manager.step_controllers(cycle_time_s);	// given the current inputs and feedback compute a control
		// device_manager.hid_input_switch();				// check for an input packet (data request/control input) handle accordingly
		switch (device_manager.input_report.read()) {
			case 64:
				blink();										// only blink when connected to a robot
				device_manager.report_switch();
				device_manager.output_report.put_int32(60, ARM_DWT_CYCCNT);
				device_manager.output_report.print();

				break;
			
			default:
				break;
		}

		device_manager.output_report.write();
		device_manager.output_report.clear();

		// device_manager.push_can();						// push data on and off the can bus
		device_manager.rm_can_ux.zero_can();					// Shutdown motors if can disconnects

		for (int i = 0; i < NUM_CAN_BUSES; i++) {
			device_manager.rm_can_ux.read_can(i);		
		}

		timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_us
		// blink();										// helpful if you think the loop is crashing (light will pause)
	}
	
	return 0;
}