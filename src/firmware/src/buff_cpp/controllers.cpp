#include "timing.h"
#include "controllers.h"

Feedback_Controller::Feedback_Controller() {
	for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
		gains[i] = 0;
	}

	min_angle = 0;
	max_angle = 0;
}

void Feedback_Controller::init(float* new_gains, float* limits) {
	for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
		gains[i] = new_gains[i];
	}

	// limits [low roll over, high roll over, low angle, high angle]
	min_angle = (2 * PI * limits[0]) + limits[2];
	max_angle = (2 * PI * limits[1]) + limits[3];
}

float Feedback_Controller::step(float* reference, float* feedback) {
	
	float u = 0;

	for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
		u += gains[i] * (reference[i] - feedback[i]);
	}

	return min(1, max(-1, u));
}

void Feedback_Controller::bound_reference(float* reference) {

	if (min_angle >= max_angle) {
		return;
	}
	
	if (min_angle > reference[0]) {
		reference[0] = min_angle;
		reference[1] = 0;
	}
	else if (max_angle < reference[0]) {
		reference[0] = max_angle;
		reference[1] = 0;
	}
}

Controller_Manager::Controller_Manager() {
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		output[i] = 0;
		biases[i] = 0;

		for (int j = 0; j < MOTOR_FEEDBACK_SIZE; j++) {
			feedback[i][j] = 0;			
			references[i][j] = 0;			
		}

		for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
			chassis_inverse_kinematics[i][j] = 0;
		}
	}

	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		input[i] = 0;			
	}
}

void Controller_Manager::reset_controller(int controller_id) {
	input[controller_id] = 0;
	output[controller_id] = 0;
	biases[controller_id] = 0;
	feedback[controller_id][0] = 0;
	feedback[controller_id][1] = 0;
	feedback[controller_id][2] = 0;
	references[controller_id][0] = 0;
	references[controller_id][1] = 0;
	references[controller_id][2] = 0;
}

void Controller_Manager::set_gain(int controller_id, int gain_id, float data){
	reset_controller(controller_id);
	controllers[controller_id].gains[gain_id] = data;
}

void Controller_Manager::init_controller(int controller_id, float* gains, float* limits){
	reset_controller(controller_id);
	controllers[controller_id].init(gains, limits);
}

void Controller_Manager::get_control_report(int controller_block, float* data) {
	int block_offset = controller_block * CAN_MOTOR_BLOCK_SIZE;
	for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE; i++) {
		data[(3 * i)] = output[i + block_offset];
		data[(3 * i) + 1] = references[i + block_offset][0];
		data[(3 * i) + 2] = references[i + block_offset][1];
	}
}

void Controller_Manager::step_motors(RM_CAN_Device* motor_index) {
	// Set the output to each motor by giving that motors feedback to a controller
	float speed = 0;
	float prev_angle = 0;
	bool new_references = timer_info_ms(2) > 10;

	// Serial.printf("outputs: ");
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {

		// only update references every 10ms
		if  (new_references) {
			timer_set(2);

			speed = 0;

			for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
				speed += chassis_inverse_kinematics[i][j] * input[j];
			}

			// integrate the speed to a postion
			references[i][0] += speed * 0.01; // 10ms
			references[i][1] = speed;

			// bound the reference state to the defined limits
			controllers[i].bound_reference(references[i]);
		}


		// motor_index[i].data[0] is the motor_angle add 2pi roll over to make an output angle
		// leave tmp[1] 0 to allow velocity control (doesn't use error like position)
		// set tmp[2] to the sum of the change in input (make gain 3 act as a damper)
		if (biases[i] == 0){
			biases[i] = motor_index[i].data[0] + (2 * PI * motor_index[i].roll_over);
		}

		prev_angle = feedback[i][0];
		feedback[i][0] = motor_index[i].data[0] + (2 * PI * motor_index[i].roll_over) - biases[i];
		feedback[i][1] = motor_index[i].data[1];
		feedback[i][2] = (feedback[i][0] - prev_angle);

		// if (i == 4) {
		// 	Serial.printf("controller: %f %f %f\n", feedback[i][0], motor_index[i].data[0], (2 * PI * motor_index[i].roll_over));
		// }

		output[i] = controllers[i].step(references[i], feedback[i]);
		// Serial.printf("%.4f, ", output[i]);
	}
	// Serial.println();
	
}

void Controller_Manager::set_input(float* control_input) {
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		input[i] = control_input[i];		
	}
}