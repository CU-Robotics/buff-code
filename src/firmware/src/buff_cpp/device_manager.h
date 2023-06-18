#include "sensors/dr16.h"
#include "sensors/revEnc.h"
#include "sensors/lsm6dsox.h"
#include "sensors/icm20649.h"
#include "sensors/refSystem.h"
#include "buff_cpp/controllers.h"
#include "robot_comms/hid_report.h"
#include "motor_drivers/rm_can_interface.h"

#ifndef BUFF_DEVICE_MANAGER_H
#define BUFF_DEVICE_MANAGER_H

float yaw_reference_buffer[20];

#define ROBOT_WAYPOINTS 5
/*
	An organizer for all the pipelines with the firmware.
	A large portion of this is HID handling (lots of switches)
	Drivers should provide a good api for this object to use them.

	The idea is that by organizing drivers here our main is 
	dummy simple and easy to adjust the scheduling.
*/

struct Device_Manager {	
	Device_Manager();

	// handlers for HID reports
	void initializer_report_handle();
	void feedback_request_handle();
	void control_input_handle();
	void sensor_request_handle();

	// More HID report handlers
	void report_switch();
	void hid_input_switch(uint32_t);

	// Non-HID related pipelines
	void push_can();
	void read_sensors();			// add functionality to read the Rev encoders (in addition to another sensor)
	void step_controllers(float);

	// Data Store
	// this is the cylinder in the flow chart 
	// drivers will put their data here
	Hid_Report input_report;	// message being read on teensy from computer (request or command)
	Hid_Report output_report;	// data being sent from teensy to computer

	// ADD new IMU here, also make sure the 
	// senor pipeline and dev manager constructors knows about it!!
	// ICM20649 chassis_imu;
	ICM20649 gimbal_imu;

	DR16 receiver;
	RefSystem ref;

	RevEnc pitchEncoder = RevEnc(5);
	RevEnc yawEncoder = RevEnc(4);
	RevEnc xOdometryEncoder = RevEnc(3);
	RevEnc yOdometryEncoder = RevEnc(2);

	RM_CAN_Interface rm_can_ux;

	Controller_Manager controller_manager;

	float robot_waypoints[2 * ROBOT_WAYPOINTS];

	// track the time since a connection was made
	float lifetime;

	// Logic for state machines (literally which case to run)
	// see implementation for more details
	int sensor_switch;
	int controller_switch;

	float yaw_ang_err;
};

#endif