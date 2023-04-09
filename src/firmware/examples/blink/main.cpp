// #include "unity.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/blink.h"

int test_setup_blink(void) {
	// test stuff
	setup_blink();
	return 0;
}

int test_blink(void) {
	// more test stuff
	blink();
	blinker_status = false;
	bool init_status = blinker_status;
	blinker_timer_mark = ARM_DWT_CYCCNT;

	timer_set(0);
	while (init_status == blinker_status) {
		blink();
	}
	// blinker should be precise to 0 microseconds
	if (abs(BLINK_RATE_US - timer_info_us(0)) > 1) {
		Serial.println("Failed blink test");
		Serial.printf("%i != %i\n", BLINK_RATE_US, timer_info_us(0));
		return 1;
	}
	// TEST_ASSERT_INT32_WITHIN(15, BLINK_RATE_US, timer_info_us(0));
	return 0;
}

int runTests(void) {
	test_setup_blink();
	int errs = test_blink();
	return errs;

	// UNITY_BEGIN();
	// RUN_TEST(test_setup_blink);
	// RUN_TEST(test_blink);
	// return UNITY_END();
}


int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};
	Serial.println("Start blink tests");
	delay(2000);

	int errors = runTests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}