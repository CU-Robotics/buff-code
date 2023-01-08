#include <Arduino.h>
#include "unity.h"
#include "sensors/dr16.cpp"
#include "buff_cpp/timing.cpp"

DR16 receiver;

void setUp() {
 	// set stuff up here
}

void tearDown() {
 	// clean stuff up here
}


void test_dr16_serial_active() {
	// check
	TEST_ASSERT(receiver.serial);
}


void loop_for(int32_t duration, bool debug) {
	int32_t test_timout = ARM_DWT_CYCCNT;

	while (DURATION_MS(test_timout, ARM_DWT_CYCCNT) < duration) {

		timer_set(0);
		receiver.read();
		timer_mark(0);

		if (debug) {
			print_control_data(receiver.data);			
		}

		timer_wait_us(0, 15000);
	}
}

void test_dr16_null_read() {
	// check
	Serial.printf("\tTesting Null input, please don't touch the controls:...\n");
	loop_for(1000, false);

	int32_t byte_sum = 0;
	for (int i = 2; i < 7; i++) {
		byte_sum += receiver.data[i];
	}

	TEST_ASSERT_EQUAL_INT32(0, abs(byte_sum));
}


void test_dr16_active_read() {
	// check
	Serial.printf("\tTesting active input, hold any button/joystick:...\n");
	delay(1500);
	
	loop_for(1000, false);

	int32_t byte_sum = 0;
	for (int i = 2; i < 7; i++) {
		byte_sum += receiver.data[i];
	}

	TEST_ASSERT_GREATER_THAN_INT32(0, abs(byte_sum));
}


int run_receiver_tests() {
	UNITY_BEGIN();
	RUN_TEST(test_dr16_serial_active);
	RUN_TEST(test_dr16_null_read);
	RUN_TEST(test_dr16_active_read);
	return UNITY_END();
}

// Runs once
void setup() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	delay(2000);

	run_receiver_tests();
}

// Runs continuously
void loop() {

}