#include "timing.h"
#include "controllers.h"

void rotate2D(float* v, float* v_tf, float angle) {
	v_tf[0] = (v[0] * cos(angle)) - (v[1] * sin(angle));
	v_tf[1] = (v[0] * sin(angle)) + (v[1] * cos(angle));
}

float vector_product(float* a, float* b, int n) {
	int ret = 0;
	for (int i = 0; i < n; i++) {
		ret += a[i] * b[i];
	}

	return ret;
}

float cross_product2D(float* a, float* b) {
	return (a[0] * b[1]) - (a[1] * b[0]);
}

void weighted_vector_addition(float* a, float* b, float k1, float k2, int n, float* output) {
	for (int i = 0; i < n; i++) {
		output[i] = (k1 * a[i]) + (k2 * b[i]);
	}
}

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
		controller_types[i] = 0;

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

void Controller_Manager::init_controller(int controller_id, int type, float* gains, float* limits){
	reset_controller(controller_id);
	controllers[controller_id].init(gains, limits);
	controller_types[controller_id] = type;
}

void Controller_Manager::get_control_report(int controller_id, float* data) {
	data[0] = output[controller_id];
	data[1] = references[controller_id][0];
	data[2] = references[controller_id][1];
	data[3] = feedback[controller_id][0];
	data[4] = feedback[controller_id][1];
	data[5] = feedback[controller_id][2];
}

void Controller_Manager::get_vel_est_report(float* data) {
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		data[i] = kee_state[i];
		data[i + REMOTE_CONTROL_LEN] = imu_state[i];
	}
}

void Controller_Manager::get_pos_est_report(float* data) {
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		data[i] = kee_imu_pos[i];
		data[i + REMOTE_CONTROL_LEN] = enc_mag_pos[i];
	}
}

void Controller_Manager::get_manager_report(float* data) {
	for (int i = 0; i < REMOTE_CONTROL_LEN ; i++) {
		data[i] = input[i];
	}
	data[REMOTE_CONTROL_LEN] = power_buffer;
}

void Controller_Manager::step_motors() {
	float ratio = 1.0;
	float power_buffer_limit_thresh = 60.0;
	float power_buffer_critical_thresh = 30.0;
	// Set the output to each motor by giving that motors feedback to a controller
	// Serial.printf("outputs: ");

	if (power_buffer < power_buffer_limit_thresh) {
		ratio = constrain((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh, 0.0, 1.0);
	}

	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		switch(controller_types[i]){
			case 0:
				output[i] = controllers[i].step(references[i], feedback[i]) * ratio;

			case 1:
				output[i] = controllers[i].step(references[i], feedback[i]);
		}
		// Serial.printf("%.4f, ", output[i]);
	}
	// Serial.println();
}

void Controller_Manager::set_reference(int controller_id) {
	float speed = 0;
	float rotated_input[2];

	// control is relative to yaw angle (gimbal heading wrt chassis)
	rotate2D(input, rotated_input, gimbal_yaw_angle);

	for (int j = 0; j < 2; j++) {
		speed += chassis_inverse_kinematics[controller_id][j] * rotated_input[j];
	}

	for (int j = 2; j < REMOTE_CONTROL_LEN; j++) {
		speed += chassis_inverse_kinematics[controller_id][j] * input[j];
	}

	// integrate the speed to a postion
	// can also set reference[x][1] = 0; (2nd gain is then a friction term)
	references[controller_id][0] += speed * 0.01; // 10ms
	references[controller_id][1] = speed;

	// bound the reference state to the defined limits
	controllers[controller_id].bound_reference(references[controller_id]);
}

void Controller_Manager::set_feedback(int controller_id, float* data, float rollover) {
	// motor_index[i].data[0] is the motor_angle add 2pi roll over to make an output angle
	// leave tmp[1] 0 to allow velocity control (doesn't use error like position)
	// set tmp[2] to the sum of the change in input (make gain 3 act as a damper)
	if (biases[controller_id] == 0){
		biases[controller_id] = data[0] + (2 * PI * rollover);
	}

	feedback[controller_id][0] = data[0] + (2 * PI * rollover) - biases[controller_id];
	feedback[controller_id][1] = data[1];

	switch(controller_types[controller_id]) {
		case 0:
			feedback[controller_id][2] = output[controller_id]; // voltage times current
			break;

		case 1:
			feedback[controller_id][2] = sin(pitch_angle + pitch_offset);
			break;

		default:
			break;
	}
}

void Controller_Manager::estimate_state(float* chassis_imu, float chassis_yaw, float dt) {

	float imu_accel_state[2];

	// compute the kee velocity estimate
	// turn motor speed feedback to robot speed feedback
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		for (int j = 0; j < MAX_NUM_RM_MOTORS; j++) {
			kee_state[i] += chassis_kinematics[i][j] * feedback[j][1];
		}
	}

	// compute the accelerometer velocity estimate
	// turn inertial measurement to robot state measurement
	rotate2D(chassis_imu, imu_accel_state, imu_offset_angle);

	// velocity of IMU relative to world in the chassis reference frame
	imu_state[0] += imu_accel_state[0] * dt;
	imu_state[1] += imu_accel_state[1] * dt;
	imu_state[2] = chassis_imu[5]; // gyro yaw
	// use other imu for the rest
	// imu_state[3] = ; // pitch
	// imu_state[4] = ; // yaw
	// imu_state[5] = ; // feeder (can leave zero)
	// imu_state[6] = ; // constant (can leave zero)

	// fuse velocity estimates (increase k1 + k2 < 1 to 'add gain')
	// float weights[REMOTE_CONTROL_LEN] = {(r * chassis_imu[5]), (r * chassis_imu[5]), 0, 0, 0, 0, 0};
	// float compensated_imu_state[REMOTE_CONTROL_LEN];
	// weighted_vector_addition(imu_state, weights, 1, -1, REMOTE_CONTROL_LEN, compensated_imu_state);
	weighted_vector_addition(imu_state, kee_state, 0.5, 0.5, REMOTE_CONTROL_LEN, kee_imu_state);

	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		kee_imu_pos[i] += kee_imu_state[i] * dt;
	}

	float x_enc_vel = 0;
	float y_enc_vel = 0;
	float raw_enc_heading[2] = {x_enc_vel, y_enc_vel};
	float enc_heading[2];

	rotate2D(raw_enc_heading, enc_heading, chassis_yaw);
	enc_mag_pos[0] += enc_heading[0] * dt;
	enc_mag_pos[1] += enc_heading[1] * dt;
	enc_mag_pos[2] = chassis_yaw;
	enc_mag_pos[3] = pitch_angle;
	enc_mag_pos[4] = gimbal_yaw_angle;
	
	// fuse position estimates (kinda pointless just use one or the other)
	weighted_vector_addition(enc_mag_pos, kee_imu_pos, 0.8, 0.2, REMOTE_CONTROL_LEN, position_est);
}

void Controller_Manager::set_input(float* control_input) {
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		input[i] = control_input[i];		
	}
}