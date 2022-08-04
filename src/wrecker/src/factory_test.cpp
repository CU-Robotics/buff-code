#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "drivers/Buff_HID.h"
#include "factories/RMMotor.h"

HID_Device hid;
rmMotor_LUT motor_lut;
rmMotor_init_data data;

//		Timing variables
unsigned long top_time;
unsigned long canRate = 5000;
unsigned long hidRate = 1000;
unsigned long cycle_time = 1000;

IntervalTimer canDumpTmr;
IntervalTimer hidtmr;


void sendCAN(){
	writeCAN(&motor_lut);
}

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	initCAN(&motor_lut);

	canDumpTmr.priority(0); // Set interval timer to handle serial reads
  	canDumpTmr.begin(sendCAN, canRate);

 //  	rmMotor* motor = RMMotor_Factory(&motor_lut, 3, 0, 1, false);
 //  	  // again we need a better data structure to store motors
	// motor_lut.LUT[0] = motor;
	// motor_lut.n_items += 1;
}


// Runs continuously
void loop() {
	top_time = micros();

	updateIO(&motor_lut);

	send_HID(&hid);
	read_HID(&hid);

	while (micros() - top_time < cycle_time){}
}

