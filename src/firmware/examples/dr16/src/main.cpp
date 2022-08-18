#include <Arduino.h>

#include "dr16.h"

DR16 receiver;

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 1000;

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

	receiver.init(0);

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
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

	blink();		

	receiver.read();	// Read from the dr16 rx

	if (micros() - top_time > cycle_time){
		Serial.print("Over the Cycle Limit: ");
		Serial.println(micros() - top_time);
	}

	while (micros() - top_time < cycle_time){}
}

