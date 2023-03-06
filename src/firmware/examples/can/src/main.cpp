#include <Arduino.h>
#include "buff_can.h"

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))

byte buff[64];       // mock HID buffer for testing
BuffCan buffcan;     // CAN thing for testing


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

void test_int16_from_can_bytes(int iters){
	/*
		  Helper to test the can things ability to convert angles.
		@param
			iters: number of iteration to test
		@return
			None
	*/

	int16_t ints[iters];
	int16_t value = 8000;

	Serial.printf("\nTesting int16_t from bytes with %d itertions:...\n", iters);

	uint32_t start = ARM_DWT_CYCCNT;
	for (int i = 0; i < iters; i++) {
		ints[i] = bytes_to_int16_t(highByte(value), lowByte(value));	
		value -= (value / iters);	
	}
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	value = 8000;

	for (int i = 0; i < iters; i++) {
		if (ints[i] != value) {
			Serial.printf("\t[%d] failed check %d != %d\n", i,
				ints[i], value);
		}
		else {
			Serial.printf("\t[%d] Check Successful\n", i);
		}
		value -= (value / iters);
	}
}

void test_can_bus_read(){
	/*
		  Helper to test the can things ability to read the can bus.
		@param
			motor_index: index of the motor
			expected: value to compare to
		@return
			None
	*/

	uint32_t start = ARM_DWT_CYCCNT;
	CAN_message_t tmp;
	buffcan.can1.read(tmp);
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	start = ARM_DWT_CYCCNT;
	buffcan.can2.read(tmp);
	stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);
}

void test_set_output(){
	/*
		  Helper to test the can things ability to set values.
		@param
			motor_index: index of the motor
			return_id: value of the return
		@return
			None
	*/
	Serial.println("\nTesting set_output:...");

	int16_t value = 8000;
	int16_t motor_command[MAX_NUM_RM_MOTORS];
	for (int i = 0; i < buffcan.num_motors; i++) {
		motor_command[i] = value;
	}
	
	uint32_t start = ARM_DWT_CYCCNT;
	buffcan.set_output(motor_command);		
	uint32_t stop = ARM_DWT_CYCCNT;
	duration_info(start, stop);

	for (int i = 0; i < buffcan.num_motors; i++) {
		int can_bus = buffcan.motor_index[i]->can_bus;
		int message_type = buffcan.motor_index[i]->message_type;
		int message_offset = buffcan.motor_index[i]->message_offset;

		if (i > buffcan.num_motors) {
			continue;
		}

		if (can_bus < 0) {
			Serial.printf("\t[%d] Invalid Device lookup:\n", i);
			print_rm_config_struct(buffcan.motor_index[i]);
			continue;
		}

		else if (buffcan.output[can_bus][message_type].buf[message_offset] != highByte(value)) {
			Serial.printf("\n\t[%d] failed upper byte check %d != %d\n", i,
				buffcan.output[can_bus][message_type].buf[message_offset], highByte(value));

			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&buffcan.output[can_bus][message_type]);
		}

		else if (buffcan.output[can_bus][message_type].buf[message_offset + 1] != lowByte(value)) {
			Serial.printf("\n\t[%d] failed lower byte check %d != %d\n", i, 
				buffcan.output[can_bus][message_type].buf[message_offset + 1], lowByte(value));
			
			Serial.printf("\tbus %d, type %d, offset %d\n", can_bus, message_type, message_offset);
			print_can_message(&buffcan.output[can_bus][message_type]);
		}	

		else {
			Serial.printf("\t[%d] Check Successful\n", i);
		}	
	}

	Serial.println("CAN1");
	print_can_message(&buffcan.output[0][0]);
	print_can_message(&buffcan.output[0][1]);
	print_can_message(&buffcan.output[0][2]);

	Serial.println("CAN2");
	print_can_message(&buffcan.output[1][0]);
	print_can_message(&buffcan.output[1][1]);
	print_can_message(&buffcan.output[1][2]);

}

void test_set_feedback(int num_motors){
	/*
		  Helper to test the can things ability to save values.
		@param
			can_bus: index of the can bus
			motor_index: index of the motor
			return_id: value of the return
		@return
			None
	*/

	Serial.printf("\nTesting set_feedback:...\n");


	for (int can_bus = 1; can_bus < 3; can_bus++) {
		int16_t value = 8000;

		uint32_t start = ARM_DWT_CYCCNT;
		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
			CAN_message_t tmp;
			tmp.id = 0x201 + i;
			tmp.buf[0] = highByte(value / (i + 1));
			tmp.buf[1] = lowByte(value / (i + 1));
			buffcan.set_feedback(can_bus, &tmp);	
		}
		uint32_t stop = ARM_DWT_CYCCNT;
		duration_info(start, stop);

		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
			int motor_id = buffcan.motor_idx_from_return(can_bus, 0x201 + i);
			if (motor_id < 0) {
				continue;
			}

			// Confirm angle we passed through the CAN message is in the feedback struct
			if (buffcan.motor_index[motor_id]->feedback->angle != ang_from_can_bytes(highByte(value / (i + 1)), lowByte(value / (i + 1)))) {
				Serial.printf("\t[%d] failed %d != %d\n", motor_id,
					buffcan.motor_index[motor_id]->feedback->angle, ang_from_can_bytes(highByte(value / (i + 1)), lowByte(value / (i + 1))));
			}
			else {
				Serial.printf("\t[%d] Check Successful\n", i);
			}
		}
	}
	Serial.println("\nMotor Config Dumb");
	for (int i = 0; i < num_motors; i++) {
		print_rm_config_struct(buffcan.motor_index[i]);
	}
}

// Runs once
void setup() {
	Serial.begin(1000000);

	while(!Serial) {}

	Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	int num_motors = 4;
	byte index[16][3] = {{1,0,1}, {1,0,2}, {2,1,1}, {2,1,6},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
						{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
	
	uint32_t start = ARM_DWT_CYCCNT;
	buffcan.set_index(index);
	uint32_t stop = ARM_DWT_CYCCNT;
	Serial.println("Index setup test:");
	duration_info(start, stop);

	if (buffcan.num_motors != num_motors) {
		Serial.printf("Failed to initialize motor index: num_motors %d != %d\n", buffcan.num_motors, num_motors);
	}

	Serial.println("Motor Config Dumb");
	for (int i = 0; i < num_motors; i++) {
		print_rm_config_struct(buffcan.motor_index[i]);
	}

	Serial.println("\nCAN Return value index");
	for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
		Serial.printf("\t[%d]\t%d\t%d\n", i, buffcan.can1_motor_index[i], buffcan.can2_motor_index[i]);
		if (buffcan.can1_motor_index[i] >= 0) {
			int can_bus = buffcan.motor_index[buffcan.can1_motor_index[i]]->can_bus;
			if (can_bus != 0) {
				Serial.printf("\t[%d] Invalid CAN bus %d != 0\n", buffcan.can1_motor_index[i], can_bus);
			}
		}

		if (buffcan.can2_motor_index[i] >= 0) {
			int can_bus = buffcan.motor_index[buffcan.can2_motor_index[i]]->can_bus;
			if (can_bus != 1) {
				Serial.printf("\t[%d] Invalid CAN bus %d != 1\n", buffcan.can2_motor_index[i], can_bus);
			}
		}
	}

	test_int16_from_can_bytes(30);

	Serial.println("\nTesting can bus read:");
	for(int i=0; i < 5; i++){
		test_can_bus_read();
	}

	test_set_output();
	test_set_feedback(num_motors);

	Serial.printf("\n\n\tDone Testing Initilization!\n\n");
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
	if (!Serial) 
		return;

	blink();	

	// unsigned long start = micros();
	// buffcan.read_can1(buff);
	// unsigned long stop = micros();
	// Serial.print("can1 read Duration: "); Serial.print(stop - start); Serial.println("us");

	// delayMicroseconds(1000);

	// start = micros();
	// buffcan.read_can2(buff);
	// stop = micros();
	// Serial.print("can2 read Duration: "); Serial.print(stop - start); Serial.println("us");
	// delayMicroseconds(1000);
}

