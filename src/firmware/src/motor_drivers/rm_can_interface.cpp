#include <FlexCAN_T4.h>

#include "buff_cpp/timing.h"
#include "rm_can_interface.h"

int16_t bytes_to_int16_t(byte upper, byte lower) {
	/*
		  Helper to convert bytes to int16_t.
		  turn this into a macro some day.
		@param
			upper: upper byte of value
			lower: lower byte of value
		@return
			value
	*/
	return int16_t(upper << 8) | lower;
}

float ang_from_can_bytes(byte b1, byte b2){
	/*
		  Getter for the angle in can bytes.
		@param
			b1: the upper byte
			b2: the lower byte 
		@return
			angle: the angle scaled to 0-36000
	*/
	return ((bytes_to_int16_t(b1, b2) / 8191) - 0.5) * 2.0 * PI;
}

void print_rm_feedback_struct(RM_Feedback_Packet* fb) {
	/*
		  print function for a feedback packet
		@param
			fb: feedback struct
		@return
			None
	*/
	Serial.printf("\tfeedback:\t\t[%f\t%f\t%f\t%i]\n", 
		fb->angle, 
		fb->RPM, 
		fb->torque, 
		ARM_DWT_CYCCNT - fb->timestamp);
}

void print_rm_config_struct(RM_CAN_Device* dev) {
	/*
		  print function for a feedback packet
		@param
			dev: CAN config struct
		@return
			None
	*/
	Serial.println("\n\t====== RM Config");
	Serial.printf("\tcan bus:\t\t%i\n\tmessage_type:\t\t%i\n\tmessage_offset:\t\t%i\n\tmotor_id:\t\t%u\n\tesc_id:\t\t\t%i\n\treturn_id:\t\t%X\n\tmotor_type:\t\t%i\n",
		dev->can_bus, dev->message_type, dev->message_offset, dev->motor_id, dev->esc_id, dev->return_id, dev->motor_type);
	print_rm_feedback_struct(dev->feedback);
}

void print_can_message(CAN_message_t* msg) {
	/*
		  main serial print the data in a can message struct.
		@param
			msg: can message struct
		@return
			None
	*/
	Serial.println("\t======");
	Serial.print("\tid: "); Serial.println(msg->id, HEX);
	Serial.print("\t\t[");
	Serial.print(msg->buf[0]);
	Serial.print("\t");
	Serial.print(msg->buf[1]);

	Serial.print("\t");
	Serial.print(msg->buf[2]);
	Serial.print("\t");
	Serial.print(msg->buf[3]);
	Serial.print("\t");
	Serial.print(msg->buf[4]);
	Serial.print("\t");
	Serial.print(msg->buf[5]);
	Serial.print("\t");
	Serial.print(msg->buf[6]);
	Serial.print("\t");
	Serial.print(msg->buf[7]);  
	Serial.println("]");
}

void prettyprint_can_message(CAN_message_t* msg) {
	/*
		  Pretty version of the can message print.
		for return messages.
		@param
			msg: can message to print
		@return
			None
	*/
	Serial.println("\t======");
	Serial.print("\tid: "); Serial.println(msg->id - 0x200, HEX);
	Serial.print("\t\t");
	Serial.print(map((uint16_t(msg->buf[0]) << 8) | uint16_t(msg->buf[1]), 0, 8191, 0, 360));
	Serial.print("\t");
	Serial.print((int16_t(msg->buf[2]) << 8) | int16_t(msg->buf[3]));
	Serial.print("\t");
	Serial.println((int16_t(msg->buf[4]) << 8) | int16_t(msg->buf[5]));
}

RM_Feedback_Packet::RM_Feedback_Packet() {
	angle = 0;
	RPM = 0;
	torque = 0;
	timestamp = ARM_DWT_CYCCNT;
}

RM_CAN_Device::RM_CAN_Device() {
	can_bus = -1;				// index of the bus the device is connected to (0 = 1, 1 = 2)
	message_type = 0;			// 0: 0x200, 1: 0x1FF, 2: 0x2FF
	message_offset = 0;			// 0-6 (always even)

	esc_id = -1;				// 0-8 the blinking light
	motor_id = -1;				// index of motor in the serialized motor structure
	motor_type = 0;				// 0: C6XX, 1: GM6020

	return_id = -1;				// actual return code

	feedback = new RM_Feedback_Packet();
}

RM_CAN_Device::RM_CAN_Device(int8_t id, byte* config) {
	can_bus = config[0] - 1;
	motor_type = config[1];
	esc_id = config[2];

	message_type = int(config[2] / 4) + config[1];			// message type = (esc ID / 4) + motor_type 
	message_offset = (2 * ((config[2] - 1) % 4));			// message offset = (esc ID % 4) - 1
	return_id = config[2] + (5 * config[1]) - 1;			// 0x200 + esc ID + motor_type_offset = rid (store as 8 bit though - 0x201)

	motor_id = id;

	feedback = new RM_Feedback_Packet();
}

RM_CAN_Interface::RM_CAN_Interface(){
	/*
		  RM_CAN_Interface constructor, initializes can busses and sets can messages and indices to defaults.
		@param
			None
		@return
			RM_CAN_Interface: Can manager code
	*/
	// start can1 and can2
	can1.begin();
	can1.setBaudRate(1000000);
	can2.begin();
	can2.setBaudRate(1000000);

	// set each message id for both can busses, not sure if it needs to be reset sometimes or what
	for (int i = 0; i < 2; i++){
		output[i][0].id = 0x200;
		output[i][1].id = 0x1FF;
		output[i][2].id = 0x2FF;
	}
	for (int j = 0; j < MAX_CAN_RETURN_IDS; j++) {
		can1_motor_index[j] = -1;
		can2_motor_index[j] = -1;
	}
}

// struct RM_CAN_Device {
//     int8_t can_bus;        // bus the device is connected to (1, 2)
//     int8_t message_type;   // 0: 0x200, 1: 0x1FF, 2: 0x2FF
//     int8_t message_offset; // 0-6 (always even)

//     int8_t esc_id;         // 1-8 the blinking light
//     int8_t motor_id;       // index of motor in the serialized motor structure
//     int8_t return_id;      // id of can message returned from device (- 0x201 to store as int8_t)

//     int8_t motor_type;     // 0: C6XX, 1: GM6020
//     RM_Feedback_Packet* feedback;
// };
void RM_CAN_Interface::set_index(int idx, byte config[3]){
	/*
		  Initialize the index for object searches.
		@param
			index: 0-16 index of motor
			config: three values each to define 
				the can packet mapping.
		@return
			None
	*/
	motor_index[idx] = RM_CAN_Device(idx, config);

	int8_t rid = motor_index[idx].return_id;

	switch(config[0] - 1) {
		case 0:
			can1_motor_index[rid] = idx;
			num_motors += 1;
			break;

		case 1:
			can2_motor_index[rid] = idx;
			num_motors += 1;
			break;

		default:
			break;
	}
}

// Some quick getters so you don't have to write
// motor_index[motor_id].feedback->angle everytime
float RM_CAN_Interface::get_motor_angle(int motor_id) {
	return motor_index[motor_id].feedback->angle;
}

float RM_CAN_Interface::get_motor_RPM(int motor_id) {
	return motor_index[motor_id].feedback->RPM;
}

float RM_CAN_Interface::get_motor_torque(int motor_id) {
	return motor_index[motor_id].feedback->torque;
}

float RM_CAN_Interface::get_motor_ts(int motor_id) {
	return motor_index[motor_id].feedback->timestamp;
}

int8_t RM_CAN_Interface::motor_idx_from_return(int8_t can_bus, int return_id){
	/*
		  Getter for the motor index.
		@param
			can_bus: can bus number (number not index)
			return_id: id of can message replied from motors
		@return
			idx: 0-MAX_NUM_MOTORS
	*/
	if (return_id - 0x201 >= 0 && return_id - 0x201 < MAX_CAN_RETURN_IDS) {
		switch (can_bus) {
			case 1: 
				return can1_motor_index[return_id - 0x201];
			case 2:
				return can2_motor_index[return_id - 0x201];

			default:
				return -1;
		}
	}    
	return -1;
}

void RM_CAN_Interface::zero_can() {
	/*
		  Zero out the values in the can packets,
		this shuts down the motors.
		@param
			None
		@return
			None
	*/
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 8; j++) {
			output[0][i].buf[j] = 0;
			output[1][i].buf[j] = 0;
		}
	}
}

void RM_CAN_Interface::write_can(){
	/*
		  Write the can output packets to the can busses.
		@param
			None
		@return
			None
	*/
	for (int i = 0; i < 3; i++){
		can1.write(output[0][i]);
		can2.write(output[1][i]);
	}
}

void RM_CAN_Interface::set_output(int16_t values[MAX_NUM_RM_MOTORS]){
	/*
		  Send the motor command from serial to the
		can packets.
		@param
			values: values to set in the can packet
		@return
			None
	*/

	int can_bus;
	int msg_type;
	int msg_offset;

	for (int i = 0; i < num_motors; i++) {
		can_bus = motor_index[i].can_bus;
		msg_type = motor_index[i].message_type;
		msg_offset = motor_index[i].message_offset;

		// The can busses are numbered 1-2 (indexed 0-1)
		output[can_bus][msg_type].buf[msg_offset] = highByte(values[i]);
		output[can_bus][msg_type].buf[msg_offset + 1] = lowByte(values[i]);
	}
}

void RM_CAN_Interface::set_feedback(int can_bus, CAN_message_t* msg){
	/*
		  Set the motors feedback into the motor index.
		@param
			can_bus: number of can bus
			msg: CAN message object
		@return
			None
	*/
	int ret_id = msg->id;

	if (ret_id > 0){
		int motor_id = motor_idx_from_return(can_bus, ret_id);

		if (motor_id >= 0) {
			motor_index[motor_id].feedback->angle = ang_from_can_bytes(msg->buf[0], msg->buf[1]);
			motor_index[motor_id].feedback->RPM = bytes_to_int16_t(msg->buf[2], msg->buf[3]);
			motor_index[motor_id].feedback->torque = bytes_to_int16_t(msg->buf[4], msg->buf[5]);
			motor_index[motor_id].feedback->timestamp = ARM_DWT_CYCCNT;
		}
	}
}

void RM_CAN_Interface::get_motor_feedback(float* data, int idx) {
	/*
		  Read the feedback values for a motor at idx.
		@param
			data: buffer for output data
			idx: index of the motor in the motor_index
		@return
			None
	*/
	if (idx >= 0 && idx < num_motors) {
		int32_t delta_ms = DURATION_MS(get_motor_ts(idx), ARM_DWT_CYCCNT);

		if (delta_ms < MOTOR_FEEDBACK_TIMEOUT){
			data[0] = get_motor_angle(idx);
			data[1] = get_motor_RPM(idx);
			data[2] = get_motor_torque(idx);
		}
		else {
			data[0] = 0;
			data[1] = 0;
			data[2] = 0;
		}
	}
	else {
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
	}
}

void RM_CAN_Interface::get_block_feedback(float* data, int id) {
	/*
		  Read the feedback values for a block of motors.
		@param
			data: buffer for output data
			id: index of the motor in the motor_index
		@return
			None
	*/
	float tmp[3];

	for (int j = 0; j < 4; j++) {
		get_motor_feedback(tmp, (4 * id) + j);
		data[3 * j] = tmp[0];
		data[(3 * j) + 1] = tmp[1];
		data[(3 * j) + 2] = tmp[2];
	}
}

void RM_CAN_Interface::read_can1(){
	/*
		  read can bus 1
		@param
			buff: buffer to save data too
		@return
			None
	*/
	CAN_message_t tmp;
	can1.read(tmp);
	set_feedback(1, &tmp);
}

void RM_CAN_Interface::read_can2(){
	/*
		  read can bus 2
		@param
			buff: buffer to save data too
		@return
			None
	*/
	CAN_message_t tmp;
	can2.read(tmp);
	set_feedback(2, &tmp);
}


