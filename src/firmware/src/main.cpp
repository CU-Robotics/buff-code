#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "drivers/rmmotor.h"
#include "drivers/buff_hid.h"
#include "algorithms/Buffers.h"


HID_Device hid;

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 2500;

bool ENABLE_HID = false;

CircularBuffer cycle_history;

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

  	cycle_history.init(10);

  	init_HID(&hid);
  	// Serial.println("Hid initialized");
}

void blink(){
	static bool status = false;
	static unsigned long t = millis();

	if (millis() - t > 250){
		status = !status;
		t = millis();
	}

	digitalWrite(LED_BUILTIN, status);
}


// Runs continuously
void loop() {
	top_time = micros();

	send_HID(&hid);
	int n = read_HID(&hid);
	if (ENABLE_HID){	// This should enable/diable the robot if there are connection issues

		blink();		

		cycle_history.push(micros() - top_time);

		if (cycle_history.mean() > 2 * cycle_time){
			cycle_history.reset();
			Serial.print("Disabling per Cycle Limit: ");
			Serial.println(micros() - top_time);
			ENABLE_HID = false;
			// should also disable the motors and sensors
		}
	}
	else {
		ENABLE_HID = n > 0;
	}

	// if (micros() - top_time > cycle_time) {
	// 	Serial.print("Overtime ");
	// 	Serial.println(micros() - top_time);
	// }
	while (micros() - top_time < cycle_time){}
}

