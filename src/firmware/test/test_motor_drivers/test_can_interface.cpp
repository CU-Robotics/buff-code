#include <Arduino.h>

#include "unity.h"

#include "buff_cpp/timing.cpp"
#include "motor_drivers/rm_can_interface.cpp"


RM_CAN_Interface rm_can_ux;

int8_t num_motors = 5; 
byte input_motor_index[16][3] = {{2,1,5}, {1,0,5}, {2,0,2}, {2,2,6},
						{2,0,6}, {0,0,0}, {0,0,0}, {0,0,0},
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
	TEST_ASSERT_EQUAL_INT8(rm_can_ux.motor_index[i].esc_type, input_motor_index[i][1]);
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
			Helper to test the can things ability to convert bytes.
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

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void test_angle_from_can_bytes() {
	/*
			Helper to test the can things ability to convert angles.
		@param
			iters: number of iteration to test
		@return
			None
	*/

	Serial.printf("\nTesting angle from bytes:...\n");

	int16_t value = mapf(0, 0, 2 * PI, 0, 8191);
	timer_set(0);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, ang_from_can_bytes(highByte(value), lowByte(value)));
	timer_mark(0);

	value = mapf(PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	TEST_ASSERT_FLOAT_WITHIN(0.0005, PI, ang_from_can_bytes(highByte(value), lowByte(value)));
	timer_mark(0);

	value = mapf(1.75 * PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	TEST_ASSERT_FLOAT_WITHIN(0.0005, 1.75 * PI, ang_from_can_bytes(highByte(value), lowByte(value)));
	timer_mark(0);

	value = mapf(2 * PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 2 * PI, ang_from_can_bytes(highByte(value), lowByte(value)));
	timer_mark(0);
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
		Serial.printf("\t[%d]\t%i\t%i\n", i, rm_can_ux.can_motor_index[0][i], rm_can_ux.can_motor_index[1][i]);

		if (rm_can_ux.can_motor_index[0][i] >= 0) {
			TEST_ASSERT_EQUAL_INT8(0, rm_can_ux.motor_index[rm_can_ux.can_motor_index[0][i]].can_bus);
		}
		if (rm_can_ux.can_motor_index[1][i] >= 0) {
			TEST_ASSERT_EQUAL_INT8(1, rm_can_ux.motor_index[rm_can_ux.can_motor_index[1][i]].can_bus);
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
	rm_can_ux.read_can(0);
	timer_mark(0);

	Serial.println("\tTesting can2 read:...");
	timer_set(0);
	rm_can_ux.read_can(1);
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

	float value = 0.00;	

	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		timer_set(0);
		rm_can_ux.set_output(i, value);    
		timer_mark(0);

		int can_bus = rm_can_ux.motor_index[i].can_bus;
		int message_type = rm_can_ux.motor_index[i].message_type;
		int message_offset = rm_can_ux.motor_index[i].message_offset;

		if (can_bus < 0) {
			print_rm_config_struct(&rm_can_ux.motor_index[i]);
			continue;
		}

		else if (rm_can_ux.output[can_bus][message_type].buf[message_offset] != highByte(int16_t(value * rm_can_ux.motor_index[i].output_scale))) {
			Serial.printf("\n\t[%d] failed upper byte check %d != %d\n", i,
				rm_can_ux.output[can_bus][message_type].buf[message_offset], highByte(int16_t(value * rm_can_ux.motor_index[i].output_scale)));

			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&rm_can_ux.output[can_bus][message_type]);
			TEST_ASSERT_EQUAL_INT8(highByte(int16_t(value * rm_can_ux.motor_index[i].output_scale)), rm_can_ux.output[can_bus][message_type].buf[message_offset]);
		}

		else if (rm_can_ux.output[can_bus][message_type].buf[message_offset + 1] != lowByte(int16_t(value * rm_can_ux.motor_index[i].output_scale))) {
			Serial.printf("\n\t[%d] failed lower byte check %d != %d\n", i, 
				rm_can_ux.output[can_bus][message_type].buf[message_offset + 1], lowByte(int16_t(value * rm_can_ux.motor_index[i].output_scale)));
			
			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&rm_can_ux.output[can_bus][message_type]);
			TEST_ASSERT_EQUAL_INT8(lowByte(int16_t(value * rm_can_ux.motor_index[i].output_scale)), rm_can_ux.output[can_bus][message_type].buf[message_offset + 1]);
		} 
	}

	Serial.println("\tCAN1");
	print_can_message(&rm_can_ux.output[0][0]);
	print_can_message(&rm_can_ux.output[0][1]);
	print_can_message(&rm_can_ux.output[0][2]);

	Serial.println("\tCAN2");
	print_can_message(&rm_can_ux.output[1][0]);
	print_can_message(&rm_can_ux.output[1][1]);
	print_can_message(&rm_can_ux.output[1][2]);

	timer_set(0);
	while (timer_info_ms(0) < 1000) {
		timer_set(1);
		rm_can_ux.write_can();
		timer_wait_us(1, 1000);
	}
	
	rm_can_ux.zero_can();
	rm_can_ux.write_can();
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

	int16_t value = int16_t(mapf(PI, 0, 2 * PI, 0, 8191));
	// generate some can messages
	CAN_message_t tmp[rm_can_ux.num_motors];
	for (int i = 0; i < rm_can_ux.num_motors; i++){
		tmp[i].id = 0x201 + rm_can_ux.motor_index[i].return_id;
		tmp[i].buf[0] = highByte(value);
		tmp[i].buf[1] = lowByte(value);
	}

	// process the messages
	timer_set(0);
	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		int can_bus = rm_can_ux.motor_index[i].can_bus;
		rm_can_ux.set_feedback(can_bus, &tmp[i]);  
	}
	timer_mark(0);

	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		int can_bus = rm_can_ux.motor_index[i].can_bus;
		int motor_id = rm_can_ux.motor_idx_from_return(can_bus, 0x201 + i);

		// motor doesn't exist
		if (motor_id < 0) {
			continue;
		}

		// Confirm angle we passed through the CAN message is in the feedback struct
		TEST_ASSERT_FLOAT_WITHIN(0.001, PI, rm_can_ux.motor_index[motor_id].data[0]);
	}
}

void test_get_feedback() {
	/*
			Helper to test the can things ability to read values.
		@param
			None
		@return
			None
	*/

	Serial.printf("\tTesting get_feedback:...\n");
		
	rm_can_ux.motor_index[0].data[0] = PI;
	rm_can_ux.motor_index[0].data[1] = 10000;
	rm_can_ux.motor_index[0].timestamp = ARM_DWT_CYCCNT;

	float tmp[3];
	timer_set(0);
	rm_can_ux.get_motor_feedback(0, tmp);
	timer_mark(0);

	TEST_ASSERT_EQUAL_FLOAT(PI, tmp[0]);
	TEST_ASSERT_EQUAL_FLOAT(10000, tmp[1]);
	TEST_ASSERT_EQUAL_FLOAT(0, tmp[2]);
}


void search_for_devices() {
	Serial.printf("\tSearching for CAN devices:...\n");

	uint32_t duration = 2000;
	rm_can_ux.zero_can();

	timer_set(0);
	while (timer_info_us(0) < duration) {
		CAN_message_t tmp;
		
		if (rm_can_ux.can2.read(tmp)) {
			int8_t motor_id = rm_can_ux.motor_idx_from_return(1, tmp.id);
			rm_can_ux.set_feedback(1, &tmp);
			if (motor_id >= 0) {
				Serial.printf("\t[%i] Found Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
			else {
				Serial.printf("\t[%i] No Device config for: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}

		if (rm_can_ux.can1.read(tmp)) {
			int8_t motor_id = rm_can_ux.motor_idx_from_return(0, tmp.id);
			rm_can_ux.set_feedback(0, &tmp);
			if (motor_id >= 0) {
				Serial.printf("\t[%i] Found Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}

		rm_can_ux.write_can();
		timer_wait_us(0, duration / 10);
	}

	timer_mark(0);

	Serial.println("\tMotor Config Dump:");
	for (int i = 0; i < num_motors; i++) {
		print_rm_config_struct(&rm_can_ux.motor_index[i]);
	}
}

int run_can_tests() {
	UNITY_BEGIN();
	RUN_TEST(index_check);
	RUN_TEST(n_motor_check);
	RUN_TEST(test_int16_from_can_bytes);
	RUN_TEST(test_angle_from_can_bytes);
	RUN_TEST(test_can_bus_indices);

	Serial.println("\tTesting can bus read:...");
	for(int i=0; i < 5; i++){
		RUN_TEST(test_can_bus_read);
	}

	RUN_TEST(test_set_output);
	RUN_TEST(test_set_feedback);
	RUN_TEST(test_get_feedback);

	RUN_TEST(search_for_devices);

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