#include "sensors/dr16.h"
#include "motor_drivers/rm_can_interface.h"

#ifndef BUFF_CONTROLLERS_H
#define BUFF_CONTROLLERS_H

/*
	A scheduler to handle all the different scenarios and report
	types. This should make checking and parsing all HID a one call
	function.
*/

struct Feedback_Controller {
	Feedback_Controller();
	void init(float*, float*);
	float step(float*, float*);
	void bound_reference(float*);

	float min_angle;
	float max_angle;
	float gains[MOTOR_FEEDBACK_SIZE];
};

struct Controller_Manager {
	Controller_Manager();
	void set_gain(int, int, float);
	void reset_controller(int);
	void init_controller(int, float*, float*);
	void get_control_report(int, float*);
	void step_motors(RM_CAN_Device*);
	void set_input(float*);

	float input[REMOTE_CONTROL_LEN];
	float output[MAX_NUM_RM_MOTORS];
	float biases[MAX_NUM_RM_MOTORS];
	float feedback[MAX_NUM_RM_MOTORS][MOTOR_FEEDBACK_SIZE];
	float references[MAX_NUM_RM_MOTORS][MOTOR_FEEDBACK_SIZE];
	float chassis_inverse_kinematics[MAX_NUM_RM_MOTORS][REMOTE_CONTROL_LEN];

	Feedback_Controller controllers[MAX_NUM_RM_MOTORS];
};

#endif