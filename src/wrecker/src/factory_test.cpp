#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "factories/RMMotor.h"

IntervalTimer canDumpTmr;


rmMotor_LUT motor_lut;
rmMotor_init_data data;

unsigned long canRate = 5000;

unsigned long top_time;
unsigned long cycle_time = 1000;


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

  	rmMotor* motor = RMMotor_Factory(&motor_lut, 3, 0, 1, false);
  	  // again we need a better data structure to store motors
	motor_lut.LUT[0] = motor;
	motor_lut.n_items += 1;
}


// Runs continuously
void loop() {
	top_time = micros();

	// switch (serial input){
	// 	case 'make-motor':
	// 		RMMotor_Factory(&motor_lut, id, motorId, canBusNum);
	// 		break;
	// 	default:
	// 		break;
	// }

	updateIO(&motor_lut);

	Serial.println(motor_lut.LUT[0]->config.byteNum);

	while (micros() - top_time < cycle_time){}
}

