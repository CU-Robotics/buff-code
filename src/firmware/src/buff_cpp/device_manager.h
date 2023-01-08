#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"

#include "robot_comms/hid_report.h"

#include "motor_drivers/rm_can_interface.h"

#ifndef BUFF_DEVICE_MANAGER_H
#define BUFF_DEVICE_MANAGER_H

/*
	A scheduler to handle all the different scenarios and report
	types. This should make checking and parsing all HID a one call
	function.
*/

struct Device_Manager {
	Device_Manager();
	Device_Manager(LSM6DSOX*, DR16*, RM_CAN_Interface*);

	// handlers
	void initializer_report_handle();
	void feedback_request_handle();

	// Data pipelines
	void report_switch();
	bool hid_input_switch();

	void push_can();
	void read_sensor();

	Hid_Report* input_report;
	Hid_Report* output_report;

	LSM6DSOX* imu;
	DR16* receiver;
	RM_CAN_Interface* rm_can_ux;
};

#endif