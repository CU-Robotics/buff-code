#include <Arduino.h>
#include "buff_hid.h"

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))

unsigned long loop_time = 2000;

HID_Packet tester;

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
	Serial.printf( "\tTimer info:\n\t%1lu cycles, %1lu ns\n", delta_cycles, delta_ns);
	return delta_ns;
}

void test_get_put() {

	Serial.printf("\nTesting HID Packet get/put:...\n");

	uint32_t start = ARM_DWT_CYCCNT;
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		tester.put(i, HID_PACKET_SIZE_BYTES - i);
	}
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	Serial.println("\n\tPacket after Put");
	tester.print_packet();

	int fails = 0;

	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		if (tester.get(i) != HID_PACKET_SIZE_BYTES - i) {
			Serial.printf("\t[%d] HID get/put failed %d != %d\n", i, tester.get(i), HID_PACKET_SIZE_BYTES - i);
			fails += 1;
		}
	}

	if (fails == 0) {
		Serial.printf("\n\tPacket set and got with no errors\n");
	}

}

void test_rget_rput() {

	Serial.printf("\nTesting HID Packet rgets/rputs:...\n");
	
	byte test_arr1[HID_PACKET_SIZE_BYTES];
	byte test_arr2[HID_PACKET_SIZE_BYTES];

	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		test_arr1[i] = HID_PACKET_SIZE_BYTES - i;
		test_arr2[i] = 0;
	}

	uint32_t start = ARM_DWT_CYCCNT;
	tester.rputs(test_arr1, 0, HID_PACKET_SIZE_BYTES);
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	Serial.println("\n\tPacket after Puts");
	tester.print_packet();

	start = ARM_DWT_CYCCNT;
	tester.rgets(test_arr2, 0, HID_PACKET_SIZE_BYTES);
	stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	int fails = 0;

	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		if (test_arr2[i] != HID_PACKET_SIZE_BYTES - i) {
			Serial.printf("\t[%d] HID rgets/rputs failed %d != %d\n", i, test_arr2[i], HID_PACKET_SIZE_BYTES - i);
			fails += 1;
		}
	}

	if (fails == 0) {
		Serial.printf("\n\tPacket set and got with no errors\n");
	}

}

void test_clear() {

	Serial.printf("\nTesting HID Packet clearing:...\n");
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		tester.put(i, HID_PACKET_SIZE_BYTES - i);
	}

	Serial.println("\tPre-Clear");
	tester.print_packet();
	Serial.println();

	uint32_t start = ARM_DWT_CYCCNT;
	tester.clear();
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	Serial.println("\n\tPost-Clear");
	tester.print_packet();

	int fails = 0;

	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		if (tester.get(i) != 0) {
			Serial.printf("\t[%d] HID clear failed %d != 0\n", i, tester.get(i));
			fails += 1;
		}
	}

	if (fails == 0) {
		Serial.printf("\n\tPacket cleared with no errors\n");
	}

}

void test_write_HID(){
	for (int i = 0; i < HID_PACKET_SIZE_BYTES; i++) {
		tester.put(i, HID_PACKET_SIZE_BYTES - i);
	}

	int8_t n = 0;
	uint32_t start = ARM_DWT_CYCCNT;
	n = tester.write();
	uint32_t stop = ARM_DWT_CYCCNT;
	Serial.printf("\nWrite Time:\n");
	duration_info(start, stop);

	if (n != 64) {
		Serial.printf("\n\tDid not write all bytes %d != 64\n", n);
	}
}

void test_read_HID(){
	tester.clear();

	int8_t n = 0;
	uint32_t start = ARM_DWT_CYCCNT;
	n = tester.read();
	uint32_t stop = ARM_DWT_CYCCNT;
	Serial.printf("\nRead Time:\n");
	duration_info(start, stop);

	if (n == 64) {
		Serial.printf("\n\tPacket read with no errors\n");
	}
}


// Runs once
void setup() {
	Serial.begin(1000000);

	while (!Serial) {}

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	test_get_put();
	test_rget_rput();
	test_clear();
	Serial.printf("\nTesting HID Packet write/read:...\n");

	test_write_HID();
	test_read_HID();

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
	unsigned long t = micros();

	blink();
	test_write_HID();
	test_read_HID();
	if (micros() - t < loop_time) {
		delayMicroseconds(loop_time - (micros() - t));
	}
}

