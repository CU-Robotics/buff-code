#include "sensors/dr16.h"
#include "motor_drivers/rm_can_interface.h"

#ifndef BUFF_CONTROLLERS_H
#define BUFF_CONTROLLERS_H

/*
	A scheduler to handle all the different scenarios and report
	types. This should make checking and parsing all HID a one call
	function.
*/
#define FEEDBACK_SIZE 3

struct Feedback_Controller {
	Feedback_Controller();
	float step(float*, float*);

	float gains[FEEDBACK_SIZE];
};

struct Controller_Manager {
	Controller_Manager();
	void set_gain(int, int, float);
	void get_control_report(int, float*);
	void step_motors(RM_CAN_Device*);
	void set_input(float*, float);

	float output[MAX_NUM_RM_MOTORS];
	float references[MAX_NUM_RM_MOTORS][FEEDBACK_SIZE];
	float chassis_inverse_kinematics[MAX_NUM_RM_MOTORS][REMOTE_CONTROL_LEN];

	Feedback_Controller controllers[MAX_NUM_RM_MOTORS];
};

#endif