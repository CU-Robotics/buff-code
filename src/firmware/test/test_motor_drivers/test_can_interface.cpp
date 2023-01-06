#include <Arduino.h>

#include "unity.h"

#include "buff_cpp/timing.h"
#include "motor_drivers/rm_can_interface.cpp"

RM_CAN_Interface rm_can_ux;

int8_t num_motors = 4; 
byte input_motor_index[16][3] = {{1,0,1}, {1,0,2}, {2,1,1}, {2,1,6},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};


void setUp() {
	// set stuff up here
}

void tearDown() {
	// clean stuff up here
}

void validate_motor(int i) {
	TEST_ASSERT_EQUAL_INT8(rm_can_ux.motor_index[i].can_bus, input_motor_index[i][0] - 1);
	TEST_ASSERT_EQUAL_INT8(rm_can_ux.motor_index[i].motor_type, input_motor_index[i][1]);
	TEST_ASSERT_EQUAL_INT8(rm_can_ux.motor_index[i].esc_id, input_motor_index[i][2]);
}

void index_check() {
	Serial.println("\tIndex setup test:...");
	timer_set(0);
	for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		rm_can_ux.set_index(i, input_motor_index[i]);    
	}
	timer_mark(0);

	validate_motor(0);
	validate_motor(1);
	validate_motor(2);
	validate_motor(3);
	validate_motor(4);
}

void n_motor_check() {
	Serial.println("\tnum motors setup check:...");
	TEST_ASSERT_EQUAL_INT8(num_motors, rm_can_ux.num_motors);
}

void test_int16_from_can_bytes() {
	/*
			Helper to test the can things ability to convert angles.
		@param
			iters: number of iteration to test
		@return
			None
	*/
	int iters = 30;
	int16_t ints[iters];
	int16_t value = 8000;

	Serial.printf("\nTesting int16_t from bytes with %d itertions:...\n", iters);

	timer_set(0);
	for (int i = 0; i < iters; i++) {
		ints[i] = bytes_to_int16_t(highByte(value), lowByte(value));  
	}
	timer_mark(0);

	for (int i = 0; i < iters; i++) {
		TEST_ASSERT_EQUAL_INT16(ints[i], value);
	}
}

void test_can_bus_indices() {
	/*
			Helper to test the can things ability to access a motor from a return id.
		@param
			motor_index: index of the motor
			expected: value to compare to
		@return
			None
	*/
	Serial.println("\tCAN message Return id index:...");
	for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
		Serial.printf("\t[%d]\t%i\t%i\n", i, rm_can_ux.can1_motor_index[i], rm_can_ux.can2_motor_index[i]);

		if (rm_can_ux.can1_motor_index[i] >= 0) {
			TEST_ASSERT_EQUAL_INT8(0, rm_can_ux.motor_index[rm_can_ux.can1_motor_index[i]].can_bus);
		}
		if (rm_can_ux.can2_motor_index[i] >= 0) {
			TEST_ASSERT_EQUAL_INT8(1, rm_can_ux.motor_index[rm_can_ux.can2_motor_index[i]].can_bus);
		}
	}
}

void test_can_bus_read() {
	/*
			Helper to test the can things ability to read the can bus.
		@param
			motor_index: index of the motor
			expected: value to compare to
		@return
			None
	*/
	Serial.println("\tTesting can1 read:...");
	timer_set(0);
	CAN_message_t tmp;
	rm_can_ux.can1.read(tmp);
	timer_mark(0);

	Serial.println("\tTesting can2 read:...");
	timer_set(0);
	rm_can_ux.can2.read(tmp);
	timer_mark(0);

}

void test_set_output() {
	/*
			Helper to test the can things ability to set values.
		@param
			None
		@return
			None
	*/
	Serial.println("\nTesting set_output:...");

	int16_t value = 8000;
	int16_t motor_command[MAX_NUM_RM_MOTORS];
	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		motor_command[i] = value;
	}
	
	timer_set(0);
	rm_can_ux.set_output(motor_command);    
	timer_mark(0);

	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		int can_bus = rm_can_ux.motor_index[i].can_bus;
		int message_type = rm_can_ux.motor_index[i].message_type;
		int message_offset = rm_can_ux.motor_index[i].message_offset;

		if (can_bus < 0) {
			print_rm_config_struct(&rm_can_ux.motor_index[i]);
			continue;
		}

		else if (rm_can_ux.output[can_bus][message_type].buf[message_offset] != highByte(value)) {
			Serial.printf("\n\t[%d] failed upper byte check %d != %d\n", i,
				rm_can_ux.output[can_bus][message_type].buf[message_offset], highByte(value));

			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&rm_can_ux.output[can_bus][message_type]);
			TEST_ASSERT_EQUAL_INT8(highByte(value), rm_can_ux.output[can_bus][message_type].buf[message_offset]);
		}

		else if (rm_can_ux.output[can_bus][message_type].buf[message_offset + 1] != lowByte(value)) {
			Serial.printf("\n\t[%d] failed lower byte check %d != %d\n", i, 
				rm_can_ux.output[can_bus][message_type].buf[message_offset + 1], lowByte(value));
			
			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&rm_can_ux.output[can_bus][message_type]);
			TEST_ASSERT_EQUAL_INT8(lowByte(value), rm_can_ux.output[can_bus][message_type].buf[message_offset + 1]);
		} 
	}

	Serial.println("CAN1");
	print_can_message(&rm_can_ux.output[0][0]);
	print_can_message(&rm_can_ux.output[0][1]);
	print_can_message(&rm_can_ux.output[0][2]);

	Serial.println("CAN2");
	print_can_message(&rm_can_ux.output[1][0]);
	print_can_message(&rm_can_ux.output[1][1]);
	print_can_message(&rm_can_ux.output[1][2]);

}

void test_set_feedback() {
	/*
			Helper to test the can things ability to save values.
		@param
			can_bus: index of the can bus
			motor_index: index of the motor
			return_id: value of the return
		@return
			None
	*/

	Serial.printf("\tTesting set_feedback:...\n");

	for (int can_bus = 1; can_bus < 3; can_bus++) {
		int16_t value = 8000;


		// generate some can messages
		CAN_message_t tmp[MAX_CAN_RETURN_IDS];
		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++){
			tmp[i].id = 0x201 + i;
			tmp[i].buf[0] = highByte(value / (i + 1));
			tmp[i].buf[1] = lowByte(value / (i + 1));
		}

		// process the messages
		timer_set(0);
		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
			rm_can_ux.set_feedback(can_bus, &tmp[i]);  
		}
		timer_mark(0);

		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
			int motor_id = rm_can_ux.motor_idx_from_return(can_bus, 0x201 + i);

			// motor doesn't exist
			if (motor_id < 0) {
				continue;
			}

			// Confirm angle we passed through the CAN message is in the feedback struct
			TEST_ASSERT_EQUAL_INT16(rm_can_ux.motor_index[motor_id].feedback->angle, ang_from_can_bytes(highByte(value / (i + 1)), lowByte(value / (i + 1))));
		}
	}

	Serial.println("\tMotor Config Dumb:");
	for (int i = 0; i < num_motors; i++) {
		print_rm_config_struct(&rm_can_ux.motor_index[i]);
	}
}

int run_can_tests() {
	UNITY_BEGIN();
	RUN_TEST(index_check);
	RUN_TEST(n_motor_check);
	RUN_TEST(test_int16_from_can_bytes);
	RUN_TEST(test_can_bus_indices);

	Serial.println("\tTesting can bus read:...");
	for(int i=0; i < 5; i++){
		RUN_TEST(test_can_bus_read);
	}

	RUN_TEST(test_set_output);
	RUN_TEST(test_set_feedback);

	return UNITY_END();
}


/**
	* For Arduino framework
	*/
void setup() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	delay(2000);

	run_can_tests();
}
void loop() {}