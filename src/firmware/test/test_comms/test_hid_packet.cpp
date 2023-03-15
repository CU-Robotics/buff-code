#include "unity.h"
#include "buff_cpp/timing.h"
#include "robot_comms/hid_report.h"


Hid_Report outgoing_report;
Hid_Report incoming_report;

void setUp() {
	// set stuff up here
}

void tearDown() {
	// clean stuff up here
}


void packet_put_test() {
	Serial.printf("\nPUT test:...\n");

	// execute the function in question
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		outgoing_report.put(i, 0xFF);
	}	
	timer_mark(0);

	// check the test
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		TEST_ASSERT_EQUAL_INT8(0xFF, outgoing_report.data[i]);
	}

}

void packet_get_test() {
	Serial.printf("\nGET test:...\n");
	// insert semi-random values 
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		outgoing_report.put(i, HID_REPORT_SIZE_BYTES - i);
	}

	// execute the function
	int8_t tmp[64];
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		tmp[i] = outgoing_report.get(i);
	}
	timer_mark(0);

	// check the output
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		// Sum error of inserted values
		TEST_ASSERT_EQUAL_INT8(tmp[i], (HID_REPORT_SIZE_BYTES - i));
	}
}

void packet_puts_gets_test() {
	Serial.printf("\nPUTS/GETS test:...\n");
	// insert semi-random values 
	byte tmp1[HID_REPORT_SIZE_BYTES];
	for (size_t i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		tmp1[i] = HID_REPORT_SIZE_BYTES - i;
	}

	timer_set(0);
	outgoing_report.rputs(tmp1, 0, HID_REPORT_SIZE_BYTES);
	timer_mark(0);

	// execute the function
	byte tmp2[HID_REPORT_SIZE_BYTES];
	timer_set(0);
	outgoing_report.rgets(tmp2, 0, HID_REPORT_SIZE_BYTES);
	timer_mark(0);

	// check the output
	TEST_ASSERT_EQUAL_INT8_ARRAY(tmp1, tmp2, HID_REPORT_SIZE_BYTES);
}

void packet_int32_test() {
	Serial.printf("\nInt32 test:...\n");
	int32_t test_value = 913;
	// insert semi-random values 
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i += 4) {
		outgoing_report.put_int32(i, test_value);
	}
	timer_mark(0);

	// check the values output from the function in question
	int32_t tmp[HID_REPORT_SIZE_BYTES / 4];
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		tmp[i] = outgoing_report.get_int32(4 * i);
	}
	timer_mark(0);

	// check the output
	TEST_ASSERT_EACH_EQUAL_INT32(test_value, tmp, HID_REPORT_SIZE_BYTES / 4);
}


void packet_float_test() {
	Serial.printf("\nFloat test:...\n");
	float test_value = 3.14159;

	// insert semi-random values 
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i += 4) {
		outgoing_report.put_float(i, test_value);
	}
	timer_mark(0);
	outgoing_report.print();

	// check the values output from the function in question
	float tmp[HID_REPORT_SIZE_BYTES / 4];
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		tmp[i] = outgoing_report.get_float(4 * i);
		Serial.printf("%f\t", tmp[i]);
		if ((i + 1) % 6 == 0 && i > 0) {
			Serial.println();
		} 
	}
	timer_mark(0);

	// check the output
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		// Sum error of inserted values
		TEST_ASSERT_EQUAL_FLOAT(tmp[i], test_value);
	}
}

int run_hid_report_tests(void) {

	UNITY_BEGIN();
	RUN_TEST(packet_put_test);
	RUN_TEST(packet_get_test);
	RUN_TEST(packet_puts_gets_test);
	RUN_TEST(packet_int32_test);
	RUN_TEST(packet_float_test);

	return UNITY_END();
}


// Runs once
int main() {
	// Serial.begin(1000000);

	// if (Serial)
	// Serial.println("-- TEENSY SERIAL START --");

	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	delay(2000);
	run_hid_report_tests();
	
	return 0;
}


