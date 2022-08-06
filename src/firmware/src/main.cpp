#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "drivers/rmmotor.h"
#include "drivers/buff_hid.h"
#include "algorithms/Buffers.h"


HID_Device hid;
Motor_LUT motor_lut;


//		Timing variables
unsigned long top_time;
unsigned long canRate = 5000;
unsigned long hidRate = 1000;
unsigned long cycle_time = 1000;

IntervalTimer canDumpTmr;
IntervalTimer hidtmr;

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


	initCAN(&motor_lut);

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
	static int canctr;
	top_time = micros();

	send_HID(&hid, &motor_lut);
	int n = read_HID(&hid, &motor_lut);
	if (ENABLE_HID){	// This should enable/diable the robot if there are connection issues

		blink();		

		if (canctr > 5){
			canctr = 0;
			//writeCAN(&motor_lut);
		}
		else {
			canctr ++;
		}

		cycle_history.push(micros() - top_time);

		if (cycle_history.mean() > 2 * cycle_time){
			cycle_history.reset();
			Serial.print("Disabling per Cycle Limit: ");
			Serial.println(micros() - top_time);
			hid.imu = MPU6050();
			hid.receiver = DR16();
			ENABLE_HID = false;
			// should also disable the motors and sensors
		}
	}
	else {
		ENABLE_HID = n > 0;
	}

	while (micros() - top_time < cycle_time){}
}

