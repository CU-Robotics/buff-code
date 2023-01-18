#include "controllers.h"

Feedback_Controller::Feedback_Controller() {
	for (int i = 0; i < FEEDBACK_SIZE; i++) {
		gains[i] = 0;
	}
}

float Feedback_Controller::step(float* reference, float* feedback) {
	float u = 0;

	// feedback[0], reference[0] are angles so wrap them
	if (feedback[0] > reference[0] + PI) {
		feedback[0] -= 2 * PI;
	}
	else if (feedback[0] < reference[0] - PI) {
		feedback[0] += 2 * PI;
	}

	for (int i = 0; i < FEEDBACK_SIZE; i++) {
		u += gains[i] * (reference[i] - feedback[i]);
	}

	return min(1, max(-1, u));
}

Controller_Manager::Controller_Manager() {
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		output[i] = 0;
		for (int j = 0; j < FEEDBACK_SIZE; j++) {
			references[i][j] = 0;			
		}

		for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
			chassis_inverse_kinematics[i][j] = 0;
		}
	}
}

void Controller_Manager::set_gain(int controller_id, int gain_id, float data){
	controllers[controller_id].gains[gain_id] = data;
}

void Controller_Manager::get_control_report(int controller_block, float* data) {
	int block_offset = controller_block * CAN_MOTOR_BLOCK_SIZE;
	for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE; i++) {
		data[i] = output[i + block_offset];
		data[i+1] = references[i + block_offset][0];
		data[i+2] = references[i + block_offset][1];
	}
}

void Controller_Manager::step_motors(RM_CAN_Device* motor_index) {
	// Set the output to each motor by giving that motors feedback to a controller
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		output[i] = controllers[i].step(references[i], motor_index[i].data);
	}
}

void Controller_Manager::set_input(float* control_input, float dt) {
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		float speed = 0;

		for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
			speed += chassis_inverse_kinematics[i][j] * control_input[j];
		}

		references[i][0] += speed * dt;

		if (references[i][0] > PI) {
			references[i][0] -= 2 * PI;
		}
		else if (references[i][0] < -PI) {
			references[i][0] += 2 * PI;
		}

		references[i][1] = speed;
	}
}