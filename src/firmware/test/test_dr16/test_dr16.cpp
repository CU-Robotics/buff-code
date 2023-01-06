#include <Arduino.h>
#include "unity.h"
#include "buff_cpp/timing.h"
#include "sensors/dr16.cpp"

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


void test_dr16_null_read() {
	// check
	Serial.printf("\tTesting Null input, please don't touch the controls:...\n");
	delay(2500);

	int32_t byte_sum = 0;
	int32_t test_timout = ARM_DWT_CYCCNT;

	while (DURATION_US(test_timout, ARM_DWT_CYCCNT) < 10000) {
		timer_set(0);
		receiver.read();
		timer_mark(0);

		for (int i = 0; i < 7; i++) {
			byte_sum += receiver.data[i];
		}

		timer_wait_us(0, 1000);
	}

	TEST_ASSERT_EQUAL_INT32(0, abs(byte_sum));
}


void test_dr16_active_read() {
	// check
	Serial.printf("\tTesting active input, try to move the robot (use the contoller):...\n");
	delay(2500);

	int32_t byte_sum = 0;
	int32_t test_timout = ARM_DWT_CYCCNT;
	
	while (DURATION_US(test_timout, ARM_DWT_CYCCNT) < 10000) {
		timer_set(0);
		receiver.read();
		timer_mark(0);

		for (int i = 0; i < 7; i++) {
			byte_sum += receiver.data[i];
		}
		print_control_data(receiver.data);

		timer_wait_us(0, 1000);
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
void loop() {}