#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
#include "buff_cpp/controllers.h"
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

	// handlers
	void initializer_report_handle();
	void feedback_request_handle();
	void control_input_handle();
	void sensor_request_handle();

	// Data pipelines
	void report_switch();
	void hid_input_switch();

	void push_can();
	void read_sensors();
	void step_controllers(float);

	Hid_Report input_report;	// message being read on teensy from computer (request or command)
	Hid_Report output_report;	// data being sent from teensy to computer

	LSM6DSOX imu;
	DR16 receiver;
	RM_CAN_Interface rm_can_ux;
	Controller_Manager controller_manager;

	int sensor_switch;
	int controller_switch;
};

#endif