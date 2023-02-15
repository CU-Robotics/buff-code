#include <FlexCAN_T4.h>

#ifndef BUFFCAN_H
#define BUFFCAN_H

#define NUM_CAN_BUSES				2
#define MOTOR_FEEDBACK_SIZE 		3
#define NUM_CAN_MESSAGE_TYPES		3
#define CAN_MOTOR_BLOCK_SIZE		4
#define MAX_NUM_RM_MOTORS			16
#define MAX_CAN_RETURN_IDS			12
#define MOTOR_FEEDBACK_TIMEOUT  	100
#define ESC_0_OUTPUT_SCALE			10000
#define ESC_1_OUTPUT_SCALE			16384
#define ESC_2_OUTPUT_SCALE			30000

#define C610   0
#define C620   1
#define GM6020 2

/*

	Initialize LUTs with motor_can_index[MAX_NUM_MOTORS][2] from HID
	  [[can bus, motor type, motor id]...] (n motors)

	Read motor commands from HID and write to output CAN packets:
	  [motor_0, motor_1... motor_15] -> CAN_Message_t[NUM_CAN_BUSSES][NUM_MESSAGE_TYPES].buf[8]

	  - Use the LUTs created in init to turn motor index i --> (can bus, message type, message offset)


	Read CAN feedback into motor feedback:
	  CAN_Message_t --> [[motor_0_pos, motor_1_pos... motor_15_pos],
						 [motor_0_vel, motor_1_vel... motor_15_vel],
						 [motor_0_trq, motor_1_trq... motor_15_trq]]

	  - Use the LUTs to transform feedback data (can bus, return ID) -> motor index i

	Get motor feedbck as bytes:
	  [[motor_0_pos, motor_1_pos... motor_15_pos],  --> bytes[32], bytes[32], bytes[32]
	   [motor_0_vel, motor_1_vel... motor_15_vel],
	   [motor_0_trq, motor_1_trq... motor_15_trq]]

	 - transform list of (int16_t/float16) into bytes
*/

struct RM_CAN_Device {
	RM_CAN_Device();
	RM_CAN_Device(int, byte*);

	int can_bus;        // index of the bus the device is connected to (0 = 1, 1 = 2)
	int message_type;   // 0: 0x200, 1: 0x1FF, 2: 0x2FF
	int message_offset; // 0-6 (always even)

	int esc_id;         // 0-8 the blinking light
	int motor_index;       // index of motor in the serialized motor structure
	int esc_type;       // 0: C610, 1: C620, 2: GM6020

	int roll_over;		// track the true position

	int return_id;    	// actual return code ( -0x201 )
	float output_scale;

	float data[MOTOR_FEEDBACK_SIZE];
	uint32_t timestamp;
};

int16_t bytes_to_int16_t(byte, byte);
float ang_from_can_bytes(byte, byte);
void print_rm_config_struct(RM_CAN_Device*);
void print_can_message(CAN_message_t*);
void prettyprint_can_message(CAN_message_t*);

struct RM_CAN_Interface {
	RM_CAN_Interface();

	// initializer
	void set_index(int, byte[3]);

	// motor getters and setters
	float get_motor_angle(int);
	float get_motor_angle(String);
	float get_motor_RPM(int);
	float get_motor_RPM(String);
	float get_motor_torque(int);
	float get_motor_torque(String);
	float get_motor_ts(int);
	float get_motor_ts(String);

	// Get the motors ID from a can msg return ID
	int8_t motor_index_from_return(int, int);

	// Disabler
	void zero_can();
	void stop_can();

	// Pipeline
	void set_output(int, float);
	void set_output(String, float);
	void set_feedback(int, CAN_message_t*);
	void get_motor_feedback(int, float*);
	void get_motor_feedback(String, float*);
	void get_block_feedback(int, float*);
	void get_block_feedback(String, float*);

	// CAN I/O
	void write_can();
	void read_can(int bus_num);

	RM_CAN_Device motor_arr[MAX_NUM_RM_MOTORS];               // support for 16 motors total (max 12 per bus)
	
	int can_motor_arr[NUM_CAN_BUSES][MAX_CAN_RETURN_IDS];  	// 2 can busses, 11 possible return values

	// CAN info https://github.com/tonton81/FlexCAN_T4/blob/master/FlexCAN_T4.h
	FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
	FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

	CAN_message_t output[NUM_CAN_BUSES][NUM_CAN_MESSAGE_TYPES];  // 2 can busses with 3 messages types, each message type has up to 4 motors.

	int num_motors;

	// Motor Aliasing
	String motorAliases[MAX_NUM_RM_MOTORS];
	bool addMotor(String alias, int motorID, int CANID, int motorType);
	int aliasToMotorID(String alias);
};

#endif




