// #include "unity.h"
#include "buff_cpp/timing.h"
#include "sensors/revEnc.h"



int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};

	RevEnc enc1 = RevEnc(4);

	Serial.println("Start rev encoder tests");
	while (1) {
		timer_set(0);
		Serial.printf("start loop\n");

		float angle1 = enc1.getAngle();
		Serial.printf("read 1\n");

		Serial.printf("value\t%f\n", angle1);
		timer_wait_us(0, 1000);
	}

	return 0;
}