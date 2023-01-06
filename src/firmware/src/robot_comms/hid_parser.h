#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
#include "robot_comms/hid_packet.h"
#include "motor_drivers/rm_can_interface.h"

#ifndef BUFFHID_PARSER_H
#define BUFFHID_PARSER_H


/*
	A scheduler to handle all the different scenarios and report
	types. This should make checking and parsing all HID a one call
	function.
*/

struct HID_Parser {
	HID_Parser();
	HID_Parser(LSM6DSOX*, DR16*, RM_CAN_Interface*);

	void parse_initializer_report();

	void spin_until_initialized();

	HID_Packet* input_report;
	HID_Packet* output_report;

	LSM6DSOX* imu;
	DR16* receiver;
	RM_CAN_Interface* rm_can_ux;
};

#endif