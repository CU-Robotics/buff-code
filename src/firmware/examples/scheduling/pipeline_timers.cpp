// #include "unity.h"
#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
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
		Serial.println("-- TEENSY SCHEDULING TEST --");
	}
}

// Master loop
int main() {											// Basically a schudeling algorithm
	setup();

	float lifetime = 0;
	timer_set(8);

	while(1) {
		timer_set(0);

		// handle any hid input output
		timer_set(6);
		device_manager.read_sensors();					// read a single sensor each call (increments the sensor id automatically)
		// timer_print(timer_info_us(6), "Sensor Read pipeline");
		
		// timer_set(6);
		device_manager.step_controllers(cycle_time_s);	// given the current inputs and feedback compute a control
		// timer_print(timer_info_us(6), "Step Controller pipeline");

		timer_set(6);
		// device_manager.hid_input_switch();				// check for an input packet (data request/control input) handle accordingly
		switch (device_manager.input_report.read()) {
			case 64:
				blink();										// only blink when connected to a robot
				device_manager.report_switch();
				timer_wait_us(8, cycle_time_us);
				lifetime += cycle_time_s;
				device_manager.output_report.put_float(60, lifetime);
				timer_set(8);

				break;
			
			default:
				timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_us if no hid available
				break;
		}

		device_manager.output_report.write();
		device_manager.output_report.clear();
		// timer_print(timer_info_us(6), "HID switch pipeline");

		// timer_set(6);
		device_manager.rm_can_ux.zero_can();							// Never write to motors in test mode
		device_manager.push_can();						// push data on and off the can bus
		// timer_print(timer_info_us(6), "CAN push pipeline");

		timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_us
		// blink();										// helpful if you think the loop is crashing (light will pause)
	}
	
	return 0;
}