#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_can.h"

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 500;


BuffCan can1;
BuffCan can2;

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

	can2.read();
	can2.write();

	// int16_t newPower = (int16_t)(sin(millis() / 1000.0) * 32767);
 //  	byte byteOne = highByte(newPower);
 //  	byte byteTwo = lowByte(newPower);

 //  	byte msg[8] = {0, 0, byteOne, byteTwo, 0, 0, 0, 0};
	// can.set(0, msg);

	blink();		

	while (micros() - top_time < cycle_time){}
}

