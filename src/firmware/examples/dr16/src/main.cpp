#include <Arduino.h>

#include "dr16.h"

DR16 receiver;

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 10000;


// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

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

	byte buffer[24];

	top_time = micros();

	blink();		

	receiver.read(buffer, 0);	// Read from the dr16 rx


	if (micros() - top_time > cycle_time){
		Serial.print("Over the Cycle Limit: ");
		Serial.println(micros() - top_time);
	}

	// for(int i = 0; i < 7; i++){
	// 	Serial.print(buffer[i]);
	// 	Serial.print(" ");
	// 	buffer[i] = 0;
	// }
	// Serial.println();

	while (micros() - top_time < cycle_time){}
}

