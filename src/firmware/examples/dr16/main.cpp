#include <Arduino.h>
#include "sensors/dr16.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/blink.h"

DR16 receiver;


void test_dr16_serial_active() {
	// check
	// TEST_ASSERT(receiver.serial);
}


void loop_for(int32_t duration, bool debug) {
	int32_t test_timout = ARM_DWT_CYCCNT;

	while (DURATION_MS(test_timout, ARM_DWT_CYCCNT) < duration) {

		timer_set(0);
		receiver.read();
		timer_mark(0);

		if (debug) {
			receiver.print_control_data();		
		}

		timer_wait_us(0, int(duration / 15));

		for ( int j = 0 ; j < REMOTE_CONTROL_LEN; j++) {
			// float tmp = receiver.data[j];
			float sum = 0;
			for (int i = 0 ; i < REMOTE_CONTROL_LEN; i++) {
				sum += receiver.data[i];
			}

			// TEST_ASSERT_MESSAGE(sum - tmp > 10, "Found more than one non-zero value");
		}
		//determine if there are wrong bits being read 
		// if(abs(receiver.data[0]) > 100)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[3]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[4]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[5]);
		// }
		// if(receiver.data[1] > 100)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[3]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[4]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[5]);
		// }
		// if(receiver.data[3] > 1)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[0]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[1]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[2]);
		// } 
		// if(receiver.data[4] > 1)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[0]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[1]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[2]);
		// } 
		// if(receiver.data[0] < -100)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[4]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[5]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[6]);
		// }
		// if(receiver.data[1] < -100)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[4]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[5]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[6]);
		// }
		// if(receiver.data[3] < -1)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[0]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[1]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[2]);
		// } 
		// if(receiver.data[4] < -1)
		// {
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[0]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[1]);
		// 	TEST_ASSERT_EQUAL_FLOAT(0, receiver.data[2]);
		// } 

	}
}

void test_dr16_null_read() {
	// check
	Serial.printf("\tTesting Null input, please don't touch the controls:...\n");
	loop_for(1000, false);

	int32_t byte_sum = 0;
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		byte_sum += receiver.data[i];
	}

	// TEST_ASSERT_EQUAL_INT32(0, abs(byte_sum));
}


void test_dr16_active_read() {
	// check
	Serial.printf("\tTesting active input, hold any button/joystick:...\n");
	delay(1500);
	
	loop_for(1000, false);

	float byte_sum = 0;

	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		byte_sum += receiver.data[i];
	}

	// TEST_ASSERT_GREATER_THAN_INT32(0, abs(byte_sum));
}

typedef union
{
	float number;
	byte bytes[4];
} FLOATBYTE_t;

void dr16_data_display() {
	Serial.printf("\tDisplaying input, press any button/joystick:...\n");

	int32_t test_timout = ARM_DWT_CYCCNT;
	int duration = 1000;

	while (DURATION_US(test_timout, ARM_DWT_CYCCNT) < duration) {

		receiver.read();

		timer_wait_us(0, int(duration / 100));
		// for (int i = 0; i < REMOTE_CONTROL_LEN; i++)
		// {
		// 	for (int j = 0; j < 8; j++) {
		// 		Serial.print(bitRead(tmp[i], j));
		// 	}

		// 	Serial.print(" ");
		// 	if ((i + 1) % 6 == 0) {
		// 		Serial.println();
		// 	}
		// }

		receiver.print_control_data();
	}
}


int run_receiver_tests() {
	test_dr16_serial_active();
	test_dr16_null_read();
	test_dr16_active_read();
	dr16_data_display();

	// UNITY_BEGIN();
	// RUN_TEST(test_dr16_serial_active);
	// RUN_TEST(test_dr16_null_read);
	// RUN_TEST(test_dr16_active_read);
	// RUN_TEST(dr16_data_display);
	// RUN_TEST(loop_for);
	return 0; //UNITY_END();
}

int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	setup_blink();
	delay(2000);
	blink();

	run_receiver_tests();

	while (1){
		blink();
		timer_set(0);
		timer_wait_us(0, 5000);
		// Serial.printf("here %i", timer_info_us(0));
	}
	return 0;
}
