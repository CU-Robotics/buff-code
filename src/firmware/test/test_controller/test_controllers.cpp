// #include <Arduino.h>
// #include "unity.h"
// #include "sensors/dr16.cpp"
// #include "buff_cpp/timing.cpp"

// BUFF_PID controller;

// void setUp() {
//  	// set stuff up here
// }

// void tearDown() {
//  	// clean stuff up here
// }


// void test_pid() {
// 	TEST_ASSERT_EQUAL_FLOAT(0, controller.error_sum());

// 	controller.set_gains(1, 0, 0);
// 	timer_set(0);
// 	float control = controller.update(1.0);
// 	TEST_ASSERT_EQUAL_FLOAT(1.0 controller.update(1.0));
// 	TEST_ASSERT_EQUAL_FLOAT(1.0 controller.error_sum());
// 	TEST_ASSERT_EQUAL_FLOAT(1.0 controller.previous_error());

// 	controller.set_gains(0, 1, 0);
// 	TEST_ASSERT_EQUAL_FLOAT(10.0 controller.update(9.0));
// 	TEST_ASSERT_EQUAL_FLOAT(10.0 controller.error_sum());
// 	TEST_ASSERT_EQUAL_FLOAT(9.0 controller.previous_error());

// 	controller.set_gains(0, 0, 1);
// 	TEST_ASSERT_EQUAL_FLOAT(1.0 controller.update(10.0));
// 	TEST_ASSERT_EQUAL_FLOAT(20.0 controller.error_sum());
// 	TEST_ASSERT_EQUAL_FLOAT(10.0 controller.previous_error());
// }


// int run_controller_tests() {
// 	UNITY_BEGIN();
// 	RUN_TEST(test_pid);
// 	return UNITY_END();
// }

// // Runs once
// void setup() {
// 	// Wait ~2 seconds before the Unity test runner
// 	// establishes connection with a board Serial interface
// 	delay(2000);

// 	run_controller_tests();
// }

// // Runs continuously
// void loop() {

// }

int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	return 0;
}