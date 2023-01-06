#include <Arduino.h>
#include "buff_hid.h"

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_MS(cycles)  ((cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  ((cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))
#define CYCLE_TIME_US 100000
#define CYCLE_TIME_MS CYCLE_TIME_US / 1000

uint32_t start = ARM_DWT_CYCCNT;
int32_t delta_us = ARM_DWT_CYCCNT;

uint32_t duration_info(uint32_t start, uint32_t stop){
	/*
		  Helper to print info about *very small* durations in
		time. Prints the cycles and time in ns.
		@param
			start: (uint32_t) value of ARM_DWT_CYCCNT at the beginning of a duration
			stop: (uint32_t) value of ARM_DWT_CYCNT at the end of a duration
		@return
			delta_ns: (uint32_t) duration in nanoseconds
	*/
	uint32_t delta_cycles = stop - start;
	uint32_t delta_ns = CYCLES_TO_NS(delta_cycles); 
	Serial.printf( "\t%1lu cycles, %1lu ns\n", delta_cycles, delta_ns);
	return delta_ns;
}

void timer_mark() {
	duration_info(start, ARM_DWT_CYCCNT);
}

HID_Packet outgoing_report;
HID_Packet incoming_report;

void packet_put_test() {
	// Announce the test
	Serial.printf("\nPUT test:...\n");

	// execute the function in question
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		outgoing_report.put(i, 0xFF);
	}	
	timer_mark();

	// check the test
	long sum = 0;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		sum += outgoing_report.data[i];
	}

	if (sum != HID_PACKET_SIZE_BYTES * 0xFF) {
		Serial.printf("\tFailed check on packet Put\n");
	}
	else {
		Serial.printf("\tCheck successful\n");
	}
}

void packet_get_test() {
	Serial.printf("\nGET test:...\n");
	// insert semi-random values 
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		outgoing_report.put(i, HID_PACKET_SIZE_BYTES - i);
	}

	// execute the function
	byte tmp[64];
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		tmp[i] = outgoing_report.get(i);
	}
	timer_mark();

	// check the output
	float sum = 0;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		// Sum error of inserted values
		sum += abs(tmp[i] - (HID_PACKET_SIZE_BYTES - i));
	}

	if (sum >= 1) {
		Serial.printf("\tFailed check on packet get\n");
	}
	else {
		Serial.printf("\tCheck successful\n");
	}
}

void packet_int32_test() {
	Serial.printf("\nInt32 test:...\n");
	// insert semi-random values 
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i += 4) {
		outgoing_report.put_int32(i, 30000);
	}
	timer_mark();

	// check the values output from the function in question
	size_t cnt = 0;
	int32_t tmp[HID_PACKET_SIZE_BYTES / 4];
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i += 4) {
		tmp[cnt] = outgoing_report.get_int32(i);
		cnt += 1;
	}
	timer_mark();

	// check the output
	float sum = 0;
	for (size_t i = 0; i < cnt; i++) {
		// Sum error of inserted values
		sum += abs(tmp[i] - 30000);
	}

	if (sum >= 1) {
		Serial.printf("\tFailed check on packet\n");
	}
	else {
		Serial.printf("\tCheck successful\n");
	}
}


void packet_float_test() {
	Serial.printf("\nFloat test:...\n");
	// insert semi-random values 
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i += 4) {
		outgoing_report.put_float(i, 3.14156);
	}
	timer_mark();

	// check the values output from the function in question
	size_t cnt = 0;
	float tmp[HID_PACKET_SIZE_BYTES / 4];
	start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i += 4) {
		tmp[cnt] = incoming_report.get_float(i);
		cnt += 1;
	}
	timer_mark();

	// check the output
	float sum = 0;
	for (size_t i = 0; i < cnt; i++) {
		// Sum error of inserted values
		sum += abs(tmp[i] - 3.14156);
	}

	if (sum >= 0.001) {
		Serial.printf("\tFailed check on packet\n");
	}
	else {
		Serial.printf("\tCheck successful\n");
	}
}

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// packet_put_test();
	// packet_get_test();
	// packet_int32_test();
	// packet_float_test();

	// Serial.printf("\n\nDone Testing HID\n\n");

	outgoing_report.put(0, 255); // idle status
	outgoing_report.put_int32(60, 0); // no connection status
}

void blink(){
	static bool status = false;
	static uint32_t mark = ARM_DWT_CYCCNT;

	if (CYCLES_TO_MS(ARM_DWT_CYCCNT - mark) > 250){
		status = !status;
		mark = ARM_DWT_CYCCNT;
	}

	digitalWrite(LED_BUILTIN, status);
}

// Runs continuously
void loop() {

	int32_t loop_timer = ARM_DWT_CYCCNT;
	
	

	while (delta_us < CYCLE_TIME_US) {
		delta_us = DURATION_US(loop_timer, ARM_DWT_CYCCNT);
	}
}
