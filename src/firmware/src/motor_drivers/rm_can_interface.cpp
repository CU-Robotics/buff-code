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

// (value / 8191) * (2 * pi), (2 * pi) / 8191 = 0.00076708
float ang_from_can_bytes(byte b1, byte b2){
	/*
		  Getter for the angle in can bytes.
		@param
			b1: the upper byte
			b2: the lower byte 
		@return
			angle: the angle
	*/
	float angle = bytes_to_int16_t(b1, b2) * 0.00076708;
	
	// if (angle > PI) {
	// 	angle -= (2 * PI);
	// }

	return angle;
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
	Serial.printf("\tcan bus:\t\t%i\n\tmessage_type:\t\t%i\n\tmessage_offset:\t\t%i\n\tmotor_index:\t\t%i\n\tesc_id:\t\t\t%i\n\treturn_id:\t\t%X\n\tesc_type:\t\t%i\n",
		dev->can_bus, dev->message_type, dev->message_offset, dev->motor_index, dev->esc_id, dev->return_id, dev->esc_type);
	Serial.printf("\tfeedback:\t\t[%f\t%f\t%f\t%i]\n", 
		dev->data[0], 
		dev->data[1], 
		dev->data[2], 
		DURATION_US(dev->timestamp, ARM_DWT_CYCCNT));
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


RM_CAN_Device::RM_CAN_Device() {
	can_bus = -1;				// index of the bus the device is connected to (0 = 1, 1 = 2)
	message_type = -1;			// 0: 0x200, 1: 0x1FF, 2: 0x2FF
	message_offset = -1;		// 0-6 (always even)

	esc_id = -1;				// 0-8 the blinking light
	motor_index = -1;				// index of motor in the serialized motor structure
	esc_type = -1;				// 0: C6XX, 1: GM6020

	roll_over = 0;				// roll over of motor

	return_id = -1;				// actual return code
	output_scale = 0;

	motor_index = -1;				// index of device in motor_arr

	data[0] = 0;				// Feedback data (position, rpm, torque)
	data[1] = 0;
	data[2] = 0;

	timestamp = ARM_DWT_CYCCNT;
}

RM_CAN_Device::RM_CAN_Device(int id, byte* config) {
	can_bus = config[0] - 1;
	esc_type = config[1];
	esc_id = config[2];

	roll_over = 0;											// roll over of motor

	message_type = int(config[2] / 4);						// message type = (esc ID / 4) ( + 1, if GM6020), [0x200, 0x1FF, 0x2FF]
	message_offset = (2 * ((config[2] - 1) % 4));			// message offset = ((esc ID - 1) % 4) * 2, (% 4 gets the message postion, * 2 because theres two bytes)
	return_id = (config[2] - 1);							// 0x200 + esc ID + esc_type_offset = rid (store as -0x201 to be more readable)

	switch (esc_type) {										// different ESCs have different setup
		case 0:
			output_scale = ESC_0_OUTPUT_SCALE;
			break;

		case 1:
			output_scale = ESC_1_OUTPUT_SCALE;
			break;

		case 2:
			message_type += 1;								// return ID for GM6020 is 0x205:0x20B
			return_id += 4;
			output_scale = ESC_2_OUTPUT_SCALE;
			break;

		default:
			break;
	}

	motor_index = id;

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;

	timestamp = ARM_DWT_CYCCNT;
}

RM_CAN_Interface::RM_CAN_Interface(){
	/*
		  RM_CAN_Interface constructor, initializes can busses and sets can messages and indices to defaults.
		@param
			None
		@return
			RM_CAN_Interface: Can manager code
	*/
	// start can
	can1.begin();
	can1.setBaudRate(1000000);

	can2.begin();
	can2.setBaudRate(1000000);

	// set each message id for both can busses, not sure if it needs to be reset sometimes or what
	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		output[i][0].id = 0x200;
		output[i][1].id = 0x1FF;
		output[i][2].id = 0x2FF;
	}

	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		for (int j = 0; j < MAX_CAN_RETURN_IDS; j++) {
			can_motor_arr[i][j] = -1;
		}
	}
}

// struct RM_CAN_Device {
//     int8_t can_bus;        // bus the device is connected to (1, 2)
//     int8_t message_type;   // 0: 0x200, 1: 0x1FF, 2: 0x2FF
//     int8_t message_offset; // 0-6 (always even)

//     int8_t esc_id;         // 1-8 the blinking light
//     int8_t motor_index;       // index of motor in the serialized motor structure
//     int8_t return_id;      // id of can message returned from device (- 0x201 to store as int8_t)

//     int8_t esc_type;     // 0: C6XX, 1: GM6020
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
	motor_arr[idx] = RM_CAN_Device(idx, config);

	int rid = motor_arr[idx].return_id;
	int can_bus = config[0] - 1;

	if(can_bus >= 0) {
		// Serial.printf("New device (can bus, motor id, return id) %i %i %i\n", can_bus, idx, rid);
		can_motor_arr[can_bus][rid] = idx;
		num_motors += 1;
	}
}

// Some quick getters so you don't have to write
// motor_arr[motor_index].angle everytime
float RM_CAN_Interface::get_motor_angle(int motor_index) {
	return motor_arr[motor_index].data[0];
}

float RM_CAN_Interface::get_motor_angle(String alias) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) return get_motor_angle(motorID);
}

float RM_CAN_Interface::get_motor_RPM(int motor_index) {
	return motor_arr[motor_index].data[1];
}

float RM_CAN_Interface::get_motor_RPM(String alias) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) return get_motor_RPM(motorID);
}

float RM_CAN_Interface::get_motor_torque(int motor_index) {
	return motor_arr[motor_index].data[2];
}

float RM_CAN_Interface::get_motor_torque(String alias) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) return get_motor_torque(motorID);
}

float RM_CAN_Interface::get_motor_ts(int motor_index) {
	return motor_arr[motor_index].timestamp;
}

float RM_CAN_Interface::get_motor_ts(String alias) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) return get_motor_ts(motorID);
}

int8_t RM_CAN_Interface::motor_index_from_return(int can_bus, int return_id) {
	/*
		  Getter for the motor index.
		@param
			can_bus: can bus index (index not number)
			return_id: id of can message replied from motors
		@return
			idx: 0-MAX_NUM_MOTORS
	*/
	if (return_id - 0x201 >= 0 && return_id - 0x201 < MAX_CAN_RETURN_IDS) {
		if (can_bus >= 0 && can_bus < NUM_CAN_BUSES) {
			return can_motor_arr[can_bus][return_id - 0x201];
		}
		// Serial.printf("Can bus invalid %i\n", can_bus);
	}
	// Serial.printf("Return id bus invalid %i\n", return_id);

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
	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 8; k++) {
				output[i][j].buf[k] = 0;
			}
		}
	}
}

void RM_CAN_Interface::write_can() {
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

void RM_CAN_Interface::set_output(int index, float value) {
	/*
		  Send the motor command from serial to the
		can packets.
		@param
			values: values to set in the can packet
		@return
			None
	*/

	int can_bus = motor_arr[index].can_bus;
	int msg_type = motor_arr[index].message_type;
	int msg_offset = motor_arr[index].message_offset;
	int16_t control_in_esc_resolution = int16_t(value * motor_arr[index].output_scale);

	// The can busses are numbered 1-2 (indexed 0-1)

	// if (msg_type == 2)
	// 	Serial.printf("Setting output for %i %i %i %f\n", can_bus, msg_type, msg_offset, value);
	
	output[can_bus][msg_type].buf[msg_offset] = highByte(control_in_esc_resolution);
	output[can_bus][msg_type].buf[msg_offset + 1] = lowByte(control_in_esc_resolution);
}

void RM_CAN_Interface::set_output(String alias, float value) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) set_output(motorID, value);
}

void RM_CAN_Interface::set_feedback(int can_bus, CAN_message_t* msg){
	/*
		  Set the motors feedback into the motor index.
		@param
			can_bus: number of can bus
			msg: CAN message object
		@return
			roll over: bool
	*/

	int motor_index = motor_index_from_return(can_bus, msg->id);

	if (motor_index >= 0) {
		// Serial.printf("Received data from %X, %i\n", msg->id - 0x201, motor_index);
		float current_angle = motor_arr[motor_index].data[0];
		float feedback_angle = ang_from_can_bytes(msg->buf[0], msg->buf[1]);
		float feedback_rpm = bytes_to_int16_t(msg->buf[2], msg->buf[3]);
		float feedback_torque = bytes_to_int16_t(msg->buf[4], msg->buf[5]);

		// need a detector for roll overs
		if (current_angle < feedback_angle - PI) {
			motor_arr[motor_index].roll_over -= 1;
		}

		else if (current_angle > feedback_angle + PI) {
			motor_arr[motor_index].roll_over += 1;
		}

		motor_arr[motor_index].data[0] = feedback_angle;
		motor_arr[motor_index].data[1] = feedback_rpm;
		motor_arr[motor_index].data[2] = feedback_torque;
		motor_arr[motor_index].timestamp = ARM_DWT_CYCCNT;
	}
}

void RM_CAN_Interface::get_motor_feedback(int idx, float* data) {
	/*
		  Read the feedback values for a motor at idx.
		@param
			data: buffer for output data
			idx: index of the motor in the motor_arr
		@return
			None
	*/
	if (idx >= 0) {		
		int delta_ms = DURATION_MS(get_motor_ts(idx), ARM_DWT_CYCCNT);

		if (delta_ms < MOTOR_FEEDBACK_TIMEOUT){
			data[0] = get_motor_angle(idx);
			data[1] = get_motor_RPM(idx);
			data[2] = get_motor_torque(idx);
			// Serial.printf("Feedback read %i, %f\n", idx, data[0]);
		}
		else {
			data[0] = 0;
			data[1] = 0;
			data[2] = 0;
			// Serial.printf("data timeout %i, %i\n", idx, delta_ms);
		}
	}
	
	else {
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		// Serial.printf("invalid index %i\n", idx);
	}
}

void RM_CAN_Interface::get_motor_feedback(String alias, float* data) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) get_motor_feedback(motorID, data);
}

void RM_CAN_Interface::get_block_feedback(int id, float* data) {
	/*
		  Read the feedback values for a block of motors.
		@param
			data: buffer for output data
			id: block number of the motor block (4 blocks with 4 motors each)
		@return
			None
	*/
	float tmp[3];
	for (int j = 0; j < 4; j++) {
		get_motor_feedback((4 * id) + j, tmp);
		data[3 * j] = tmp[0];
		data[(3 * j) + 1] = tmp[1];
		data[(3 * j) + 2] = tmp[2];
	}
}

void RM_CAN_Interface::get_block_feedback(String alias, float* data) {
	int motorID = aliasToMotorID(alias);
	if (motorID != -1) get_block_feedback(motorID, data);
}

void RM_CAN_Interface::read_can(int bus_num){
	/*
		  read can bus, use timer 3 as timeout
		@param
			bus_num: index of the can bus
		@return
			None
	*/
	timer_set(3);
	CAN_message_t tmp;
	switch (bus_num) {
		case CAN1:
			while (can1.read(tmp) && timer_info_us(3) < 10) {
				set_feedback(bus_num, &tmp);
			}
			break;

		case CAN2:
			while (can2.read(tmp)) {
				set_feedback(bus_num, &tmp);
			}
			break;

		// case CAN3:
		// 	while (can3.read(tmp)) {
		// 		set_feedback(bus_num, &tmp);
		// 	}
		// 	break;

		default:
			break;
	}
}

bool RM_CAN_Interface::addMotor(String alias, int motorID, int CANID, int motorType) {
	byte config[3] = {CANID, motorType, motorID};
	int index = motorID + ((CANID - 1) * 8) - 1;
	set_index(index, config);
	motorAliases[index] = alias;
	Serial.println("Error adding motor! All RM device slots are claimed.");
	return false;
}

// TODO: Optimize (binary insertion + sort?)
int RM_CAN_Interface::aliasToMotorID(String alias) {
	for (int i = 0; i < sizeof(motorAliases); i++) if (motorAliases[i] == alias) return i;
	return -1;
}