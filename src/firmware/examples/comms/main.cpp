// #include "unity.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
#include "robot_comms/hid_report.h"


Hid_Report outgoing_report;
Hid_Report incoming_report;


int packet_put_test() {
	int errors = 0;
	Serial.printf("\nPUT test:...\n");

	// execute the function in question
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		outgoing_report.put(i, 0xFF);
	}	
	timer_mark(0);

	// check the test
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		errors += int_eq(0xFF, outgoing_report.data[i], "failed packet insert");
	}
	return errors;
}

int packet_get_test() {
	int errors = 0;
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
		errors += int_eq(tmp[i], (HID_REPORT_SIZE_BYTES - i), "failed packet read");
	}

	return errors;
}

int packet_puts_gets_test() {
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
	return bytes_eq(tmp1, tmp2, HID_REPORT_SIZE_BYTES, "Failed Puts-Gets");
}

int packet_int32_test() {
	Serial.printf("\nInt32 test:...\n");
	int32_t test_value = 913;
	// insert semi-random values 
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i += 4) {
		outgoing_report.put_int32(i, test_value);
	}
	timer_mark(0);

	// check the values output from the function in question
	int tmp[HID_REPORT_SIZE_BYTES / 4];
	timer_set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		tmp[i] = outgoing_report.get_int32(4 * i);
	}
	timer_mark(0);

	// check the output
	return ints_eq(int(test_value), tmp, HID_REPORT_SIZE_BYTES / 4, "Failed int32_t test");
}


int packet_float_test() {
	int errors = 0;
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
		errors += float_eq(tmp[i], test_value, "Failed float test");
	}

	return errors;
}

int run_hid_report_tests(void) {

	int errors = 0;
	errors += packet_put_test();
	errors += packet_get_test();
	errors += packet_puts_gets_test();
	errors += packet_int32_test();
	errors += packet_float_test();

	return errors;
}


// Runs once
int main() {
	// Serial.begin(1000000);

	// if (Serial)
	// Serial.println("-- TEENSY SERIAL START --");

	while (!Serial) {};
	Serial.println("Start comms tests");

	int errors = run_hid_report_tests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}


