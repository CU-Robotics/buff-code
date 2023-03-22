#include "sensors/dr16.h"
#include "motor_drivers/rm_can_interface.h"

#ifndef BUFF_CONTROLLERS_H
#define BUFF_CONTROLLERS_H


void rotate2D(float*, float*, float);
float vector_product(float*, float*, int);
float cross_product2D(float*, float*);
void weighted_vector_addition(float*, float*, float, float, int, float*);

/*
	Break this out into two controllers
	- Gravity compensated
	- Power Limited
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

/*
	This object is meant to run the control pipeline
	it allows easy use of the DM store to update inputs
	and feedback, then uses HID initializers to setup 
	control laws.
*/
struct Controller_Manager {
	Controller_Manager();
	void set_gain(int, int, float);
	void reset_controller(int);
	void init_controller(int, int, float*, float*);
	void get_control_report(int, float*);
	void get_vel_est_report(float*);
	void get_pos_est_report(float*);
	void get_manager_report(float*);
	void step_motors();
	void set_input(float*);
	void set_feedback(int, float*, float);
	void estimate_state(float*, float, float);
	void set_reference(int);

	float power_buffer;

	float pitch_angle;
	float pitch_offset;
	float gimbal_yaw_angle;

	float imu_offset_angle;

	float xy_odom_enc[2];
	float gimbal_enc[2];

	float input[REMOTE_CONTROL_LEN];
	float output[MAX_NUM_RM_MOTORS];
	float biases[MAX_NUM_RM_MOTORS];
	float feedback[MAX_NUM_RM_MOTORS][MOTOR_FEEDBACK_SIZE];
	float references[MAX_NUM_RM_MOTORS][MOTOR_FEEDBACK_SIZE];
	float chassis_kinematics[REMOTE_CONTROL_LEN][MAX_NUM_RM_MOTORS];
	float chassis_inverse_kinematics[MAX_NUM_RM_MOTORS][REMOTE_CONTROL_LEN];

	// add sensor weight vector for sensor fusion where sum(a) = 1, 
	// then state_est = (a1*m1) + (a2*m2) + (a3*m3), tune weights to remove
	// noise. m1 can be a linear transform from a sensor reading to a state. ie mi = R_i * measurement_i
	// Assume sensor readings are filtered by the drivers that read them.

	// velocity estimates
	float kee_state[REMOTE_CONTROL_LEN];
	float imu_state[REMOTE_CONTROL_LEN];
	float kee_imu_state[REMOTE_CONTROL_LEN];

	// postional estimates
	float kee_imu_pos[REMOTE_CONTROL_LEN];
	float enc_mag_pos[REMOTE_CONTROL_LEN];
	float position_est[REMOTE_CONTROL_LEN];

	int controller_types[MAX_NUM_RM_MOTORS];
	
	Feedback_Controller controllers[MAX_NUM_RM_MOTORS];
};

#endif