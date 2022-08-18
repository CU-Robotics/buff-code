#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_can.h"

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 1000;

BuffCan can;

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
	top_time = micros();


	can.read();


	int16_t newPower = (int16_t)(1.0 * 16384);
  	byte byteOne = highByte(newPower);
  	byte byteTwo = lowByte(newPower);

	// can.set(1, 0, 1, byteOne, byteTwo);
	// can.set(1, 0, 1, byteOne, byteTwo);
	can.set(1, 2, 1, byteOne, byteTwo);


	blink();		
	can.write();

	while (micros() - top_time < cycle_time){}
}

