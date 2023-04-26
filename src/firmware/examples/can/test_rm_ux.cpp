#include <Arduino.h>

// #include "unity.h"

#include "buff_cpp/loggers.h"
#include "buff_cpp/timing.h"
#include "motor_drivers/rm_can_interface.h"

#define VERBOSITY 1

#define NUM_MOTORS 3

#define TARGET1_CAN_BUS 	2
#define TARGET1_ESC_TYPE 	1
#define TARGET1_ESC_ID 		8
#define TARGET1_MSG_TYPE	1
#define TARGET1_MSG_OFF		6
#define TARGET1_RID			7

#define TARGET2_CAN_BUS 	2
#define TARGET2_ESC_TYPE 	1
#define TARGET2_ESC_ID 		3
#define TARGET2_MSG_TYPE	0
#define TARGET2_MSG_OFF		2
#define TARGET2_RID			2


#define TARGET3_CAN_BUS 	2
#define TARGET3_ESC_TYPE 	0
#define TARGET3_ESC_ID 		6
#define TARGET3_MSG_TYPE	1
#define TARGET3_MSG_OFF		4
#define TARGET3_RID			5

#define TARGET4_CAN_BUS 	0
#define TARGET4_ESC_TYPE 	0
#define TARGET4_ESC_ID 		0
#define TARGET4_MSG_TYPE	0
#define TARGET4_MSG_OFF		0
#define TARGET4_RID			-1


RM_CAN_Interface rm_can_ux;

int num_motors = 4; 
byte input_motor_arr[16][3] = {{TARGET1_CAN_BUS, TARGET1_ESC_TYPE, TARGET1_ESC_ID}, 
								{TARGET2_CAN_BUS, TARGET2_ESC_TYPE, TARGET2_ESC_ID},
								{TARGET3_CAN_BUS, TARGET3_ESC_TYPE, TARGET3_ESC_ID},
								{TARGET4_CAN_BUS, TARGET4_ESC_TYPE, TARGET4_ESC_ID},
								{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
								{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
								{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};

int expected_msg_type[16] = {TARGET1_MSG_TYPE, TARGET2_MSG_TYPE, TARGET3_MSG_TYPE, TARGET4_MSG_TYPE, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0};

int expected_msg_off[16] = {TARGET1_MSG_OFF, TARGET2_MSG_OFF, TARGET3_MSG_OFF, TARGET4_MSG_OFF, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0};

int expected_rid[16] = {TARGET1_RID, TARGET2_RID, TARGET3_RID, TARGET4_RID, -1, -1, -1, -1,
						-1, -1, -1, -1, -1, -1, -1, -1};


int validate_motor(int i) {
	int n = 0;
	if (VERBOSITY > 1) {
		Serial.printf("Validating motor %i\n", i);		
	}

	n += int_eq(rm_can_ux.motor_arr[i].can_bus, input_motor_arr[i][0], "Can init failed");
	n += int_eq(rm_can_ux.motor_arr[i].esc_type, input_motor_arr[i][1], "esc init failed");
	n += int_eq(rm_can_ux.motor_arr[i].esc_id, input_motor_arr[i][2], "esc init failed");
	n += int_eq(rm_can_ux.motor_arr[i].message_type, expected_msg_type[i], "message type init failed");
	n += int_eq(rm_can_ux.motor_arr[i].message_offset, expected_msg_off[i], "message offset init failed");
	n += int_eq(rm_can_ux.motor_arr[i].return_id, expected_rid[i], "rid init failed");

	return n;
}

int index_check() {
	if (VERBOSITY) {
		Serial.println("Index setup test:...");
	}

	timer_set(0);
	for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		rm_can_ux.set_index(i, input_motor_arr[i]);    
	}

	if (VERBOSITY > 1) {
		timer_mark(0);
	}
	

	int errors = 0;
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		errors += validate_motor(i);
	}

	return errors;
}

int n_motor_check() {
	// Serial.println("\tnum motors setup check:...");
	return int_eq(NUM_MOTORS, rm_can_ux.num_motors, "Num motors failed");
	// TEST_ASSERT_EQ_INT8(num_motors, rm_can_ux.num_motors);
}

int test_int16_from_can_bytes() {
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

	if (VERBOSITY) {
		Serial.printf("Testing int16_t from bytes with %d itertions:...", iters);		
	}

	timer_set(0);
	for (int i = 0; i < iters; i++) {
		ints[i] = bytes_to_int16_t(highByte(value * i / iters), lowByte(value * i / iters));  
	}

	if (VERBOSITY > 1)
		timer_mark(0);

	int errors = 0;
	for (int i = 0; i < iters; i++) {
		// TEST_ASSERT_EQ_INT16(ints[i], value);
		errors += int_eq(ints[i], value * i / iters, "Failed to convert bytes to value");
	}
	return errors;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int test_angle_from_can_bytes() {
	/*
			Helper to test the can things ability to convert angles.
		@param
			iters: number of iteration to test
		@return
			None
	*/
	int errors = 0;
	if (VERBOSITY) {
		Serial.printf("\nTesting angle from bytes:...\n");
	}

	int16_t value = mapf(0, 0, 2 * PI, 0, 8191);
	timer_set(0);
	errors += float_eq(0, ang_from_can_bytes(highByte(value), lowByte(value)), "Angle conversion wrong");
	if (VERBOSITY > 1)
		timer_mark(0);

	value = mapf(PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	errors += float_eq(PI, ang_from_can_bytes(highByte(value), lowByte(value)), "Angle conversion wrong");
	if (VERBOSITY > 1)
		timer_mark(0);

	value = mapf(1.75 * PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	errors += float_eq(1.75 * PI, ang_from_can_bytes(highByte(value), lowByte(value)), "Angle conversion wrong");
	if (VERBOSITY > 1)
		timer_mark(0);

	value = mapf(2 * PI, 0, 2 * PI, 0, 8191);
	timer_set(0);
	errors += float_eq(2 * PI, ang_from_can_bytes(highByte(value), lowByte(value)), "Angle conversion wrong");
	if (VERBOSITY > 1)
		timer_mark(0);

	return errors;
}

int test_can_bus_indices() {
	/*
			Helper to test the can things ability to access a motor from a return id.
		@param
			motor_arr: index of the motor
			expected: value to compare to
		@return
			None
	*/
	int errors = 0;
	if (VERBOSITY) {
		Serial.println("CAN message Return id index:...");
	}

	for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
		Serial.printf("\t[%d]\t%i\t%i\n", i, rm_can_ux.can_motor_arr[0][i], rm_can_ux.can_motor_arr[1][i]);

		if (rm_can_ux.can_motor_arr[0][i] >= 0) {
			errors += int_eq(0, rm_can_ux.motor_arr[rm_can_ux.can_motor_arr[0][i]].can_bus - 1, "Failed return id look up");
		}
		if (rm_can_ux.can_motor_arr[1][i] >= 0) {
			errors += int_eq(1, rm_can_ux.motor_arr[rm_can_ux.can_motor_arr[1][i]].can_bus - 1, "Failed return id look up");
		}
	}
	return errors;
}

void test_can_bus_read() {
	/*
			Helper to test the can things ability to read the can bus.
		@param
			motor_arr: index of the motor
			expected: value to compare to
		@return
			None
	*/
	if (VERBOSITY > 1) {
		Serial.println("\tTesting CAN1 read:...");
		timer_set(0);
	}
	
	rm_can_ux.read_can(1);

	if (VERBOSITY > 1) {
		timer_mark(0);
	}
	

	if (VERBOSITY > 1) {
		Serial.println("\tTesting CAN2 read:...");
		timer_set(0);
	}
	
	rm_can_ux.read_can(2);

	if (VERBOSITY > 1) {
		timer_mark(0);
	}

}

int test_set_output() {
	/*
			Helper to test the can things ability to set values.
		@param
			None
		@return
			None
	*/
	if (VERBOSITY) {
		Serial.println("Testing set_output:...");
	}

	float value = -0.05;	
	int errors = 0;

	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		timer_set(0);
		rm_can_ux.set_output(i, value);    
		if (VERBOSITY > 1) {
			timer_mark(0);
		}

		int can_bus = rm_can_ux.motor_arr[i].can_bus;
		int message_type = rm_can_ux.motor_arr[i].message_type;
		int message_offset = rm_can_ux.motor_arr[i].message_offset;

		if (can_bus < 1 && VERBOSITY > 1) {
			print_rm_config_struct(&rm_can_ux.motor_arr[i]);
			continue;
		}

		if (VERBOSITY > 1) {
			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&rm_can_ux.output[can_bus - 1][message_type]);
		}

		errors += int_eq(highByte(int16_t(value * rm_can_ux.motor_arr[i].output_scale)), rm_can_ux.output[can_bus - 1][message_type].buf[message_offset], "Output conversion failed");
		errors += int_eq(lowByte(int16_t(value * rm_can_ux.motor_arr[i].output_scale)), rm_can_ux.output[can_bus - 1][message_type].buf[message_offset + 1], "Output conversion failed");
	}

	if (VERBOSITY) {
		Serial.println("\tCAN1");
		print_can_message(&rm_can_ux.output[0][0]);
		print_can_message(&rm_can_ux.output[0][1]);
		print_can_message(&rm_can_ux.output[0][2]);

		Serial.println("\tCAN2");
		print_can_message(&rm_can_ux.output[1][0]);
		print_can_message(&rm_can_ux.output[1][1]);
		print_can_message(&rm_can_ux.output[1][2]);
	}

	timer_set(0);
	while (timer_info_ms(0) < 100) {
		timer_set(1);
		rm_can_ux.write_can();
		rm_can_ux.read_can(1);
		rm_can_ux.read_can(2);
		timer_wait_us(1, 1000);
	}

	if (VERBOSITY) {
		print_rm_config_struct(&rm_can_ux.motor_arr[0]);
	}
	
	rm_can_ux.zero_can();
	rm_can_ux.write_can();

	return errors;
}

int test_set_feedback() {
	/*
			Helper to test the can things ability to save values.
		@param
			can_bus: index of the can bus
			motor_arr: index of the motor
			return_id: value of the return
		@return
			None
	*/

	if (VERBOSITY) {
		Serial.printf("Testing set_feedback:...\n");
	}

	int16_t value = int16_t(mapf(PI, 0, 2 * PI, 0, 8191));
	// generate some can messages
	CAN_message_t tmp[rm_can_ux.num_motors];
	for (int i = 0; i < rm_can_ux.num_motors; i++){
		tmp[i].id = 0x201 + rm_can_ux.motor_arr[i].return_id;
		tmp[i].buf[0] = highByte(value);
		tmp[i].buf[1] = lowByte(value);
	}

	// process the messages
	timer_set(0);
	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		int can_bus = rm_can_ux.motor_arr[i].can_bus;
		rm_can_ux.set_feedback(can_bus, &tmp[i]);  
	}
	if (VERBOSITY > 1) {
		timer_mark(0);
	}

	int errors = 0;

	for (int i = 0; i < rm_can_ux.num_motors; i++) {
		int can_bus = rm_can_ux.motor_arr[i].can_bus;
		int motor_id = rm_can_ux.motor_index_from_return(can_bus, 0x201 + i);

		// motor doesn't exist
		if (motor_id < 0) {
			continue;
		}

		// Confirm angle we passed through the CAN message is in the feedback struct
		// TEST_ASSERT_FLOAT_WITHIN(0.001, PI, rm_can_ux.motor_arr[motor_id].data[0]);
		errors += float_eq(PI, rm_can_ux.motor_arr[motor_id].data[0], "Failed to parse feedback");
	}
	return errors;
}

int test_get_feedback() {
	/*
			Helper to test the can things ability to read values.
		@param
			None
		@return
			None
	*/

	if (VERBOSITY)
	Serial.printf("Testing get_feedback:...\n");
		
	rm_can_ux.motor_arr[0].data[0] = PI;
	rm_can_ux.motor_arr[0].data[1] = 10000;
	rm_can_ux.motor_arr[0].timestamp = ARM_DWT_CYCCNT;

	float tmp[3];
	timer_set(0);
	rm_can_ux.get_motor_feedback(0, tmp);
	if (VERBOSITY > 1) {
		timer_mark(0);
	}

	// TEST_ASSERT_EQ_FLOAT(PI, tmp[0]);
	// TEST_ASSERT_EQ_FLOAT(10000, tmp[1]);
	// TEST_ASSERT_EQ_FLOAT(0, tmp[2]);
	return 0;
}


void search_for_devices() {
	if (VERBOSITY)
		Serial.printf("Searching for CAN devices:...\n");

	uint32_t duration = 2000;
	rm_can_ux.zero_can();

	timer_set(0);
	while (timer_info_us(0) < duration) {
		CAN_message_t tmp;
		
		if (rm_can_ux.can2.read(tmp)) {
			int8_t motor_id = rm_can_ux.motor_index_from_return(2, tmp.id); // make sure can bus is correct
			rm_can_ux.set_feedback(2, &tmp);
			if (motor_id >= 0 && VERBOSITY) {
				Serial.printf("\t[%i] Found Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
			else if (VERBOSITY){
				Serial.printf("\t[%i] No Device config for: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}

		if (rm_can_ux.can1.read(tmp)) {
			int8_t motor_id = rm_can_ux.motor_index_from_return(0, tmp.id);
			rm_can_ux.set_feedback(0, &tmp);
			if (motor_id >= 0 && VERBOSITY) {
				Serial.printf("\t[%i] Found Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}

		rm_can_ux.write_can();
		timer_wait_us(0, duration / 10);
	}

	if (VERBOSITY > 1) {
		timer_mark(0);
		Serial.println("\tMotor Config Dump:");
		for (int i = 0; i < NUM_MOTORS; i++) {
			print_rm_config_struct(&rm_can_ux.motor_arr[i]);
		}
	}
}

int run_can_tests() {
	// UNITY_BEGIN();
	int errors = 0;
	errors += index_check();
	errors += n_motor_check();
	errors += test_int16_from_can_bytes();
	errors += test_angle_from_can_bytes();
	errors += test_can_bus_indices();

	Serial.println("\tTesting can bus read:...");
	for(int i=0; i < 5; i++){
		test_can_bus_read();
	}

	errors += test_set_output();
	errors += test_set_feedback();
	errors += test_get_feedback();

	search_for_devices();

	return errors;
	// return UNITY_END();
}


/**
	* For Arduino framework
	*/
int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};
	Serial.println("Start can ux tests");
	delay(2000);

	int errors = run_can_tests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}












