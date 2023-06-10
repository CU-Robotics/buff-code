#include "buff_cpp/timing.h"
#include "buff_cpp/controllers.h"

void odom_diff(float* odom_curr, float* odom_prev, float* output) {
	float diff[3];
	diff[0] = odom_curr[0] - odom_prev[0];
	diff[1] = odom_curr[1] - odom_prev[1];
	if (diff[0] < -180) diff[0] += 360;
	if (diff[0] > 180) diff[0] -= 360;
	if (diff[1] < -180) diff[1] += 360;
	if (diff[1] > 180) diff[1] -= 360;
	diff[0] = (diff[0] * (PI/180)) * .048;
	diff[1] = (diff[1] * (PI/180)) * .048;

}

float wrap_angle(float angle) {
	while (angle >= PI) {
		angle -= 2 * PI;
	}

	while (angle < -PI) {
		angle += 2 * PI;
	}

	return angle;
}

float wrap_error(float error) {
	bool wrapped = false;
	while (error >= PI) {
		error -= 2 * PI;
		wrapped = true;
	}

	while (error < -PI) {
		error += 2 * PI;
		wrapped = true;
	}

	if (wrapped) error *= -1;

	return error;
}

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

void n_weighted_vector_addition(float* a, float* b, float* k1, float* k2, int n, float* output) {
	for (int i = 0; i < n; i++) {
		output[i] = (k1[i] * a[i]) + (k2[i] * b[i]);
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
	timer_set(4);
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

	for (int i = 0; i < MAX_REV_ENCODERS; i++) {
		if (encoder_bias[i] == 0) {
			encoder_bias[i] = encoders[i];
		}
	}

	imu_yaw.set_gain(0.9);
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
	motor_filters[controller_id].set_gain(0.0);
}

void Controller_Manager::set_gain(int controller_id, int gain_id, float data){
	reset_controller(controller_id);
	controllers[controller_id].gains[gain_id] = data;
}

void Controller_Manager::init_controller(int controller_id, int type, float* gains, float* limits, float filter_gain){
	reset_controller(controller_id);
	controllers[controller_id].init(gains, limits);
	controller_types[controller_id] = type;
	motor_filters[controller_id].set_gain(filter_gain);
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
		data[i] = autonomy_goal[i];
		data[i + REMOTE_CONTROL_LEN] = enc_odm_pos[i];
	}
}

void Controller_Manager::get_manager_report(float* data) {
	for (int i = 0; i < REMOTE_CONTROL_LEN ; i++) {
		data[i] = input[i];
	}
	data[REMOTE_CONTROL_LEN] = power_buffer;
	data[REMOTE_CONTROL_LEN+1] = projectile_speed;
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
		int ctrl_type = controller_types[i]; // handle coupled motors
		if (ctrl_type < 0) {
			ctrl_type = controller_types[-controller_types[i]];
		}

		switch(ctrl_type){
			case 0:
			case 1:
			case 3:
			case 4:
			case 5:
				output[i] = controllers[i].step(references[i], feedback[i]);
				break;

			case 2:
				output[i] = controllers[i].step(references[i], feedback[i]) * ratio;
				break;

			case 6:
				output[i] = controllers[i].step(references[i], feedback[i]);
				if (output[i] > 0.7) output[i] = 0.7;
				if (output[i] < -0.7) output[i] = -0.7;
				break;

			default:
				break;
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

	int ctrl_type = controller_types[controller_id];
	if (ctrl_type < 0) {		// handle coupled controllers
		ctrl_type = controller_types[-ctrl_type];
	}

	// integrate the speed to a postion
	// can also set reference[x][1] = 0; (2nd gain is then a friction term)
	references[controller_id][0] += speed * 0.01; // 10ms this can't be hardcoded

	if (ctrl_type != 1 && ctrl_type != 3) {
		references[controller_id][1] = speed;
	}

	// bound the reference state to the defined limits
	controllers[controller_id].bound_reference(references[controller_id]);

	// for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
	// 	Serial.print(references[i][1]);
	// 	Serial.print(", ");
	// }
	// Serial.println();

	// if (controller_id == 0) {
	// 	Serial.printf("%f %f %f\n", gimbal_yaw_angle, rotated_input[0], speed);
	// }
}

void Controller_Manager::set_feedback(int controller_id, float* data, float rollover) {
	// motor_index[i].data[0] is the motor_angle add 2pi roll over to make an output angle
	// leave tmp[1] 0 to allow velocity control (doesn't use error like position)
	// set tmp[2] to the sum of the change in input (make gain 3 act as a damper)
	if (biases[controller_id] == 0 && controller_id != 4 && controller_id != 5) {
		biases[controller_id] = data[0] + (2 * PI * rollover);
		references[controller_id][0] = 0;
	}

	feedback[controller_id][0] = data[0] + (2 * PI * rollover) - biases[controller_id];
	feedback[controller_id][1] = motor_filters[controller_id].filter(data[1] * 2 * PI / 60);
	feedback[controller_id][2] = data[2];

	switch(controller_types[controller_id]) {
		case 0:
		case 1:
		case 2:
		case 6:
			// feedback[controller_id][2] = output[controller_id] * feedback[controller_id][1]; // voltage times current
			break;

		case 3:
			feedback[controller_id][2] = -sin((feedback[controller_id][0] / 4.5) + (PI / 2.75));
			break;

		case 4:
			feedback[controller_id][0] = -gimbal_pitch_angle * 152 / 17.0;
			feedback[controller_id][2] = cos(-gimbal_pitch_angle);
			break;

		case 5:
			feedback[controller_id][0] = gimbal_pitch_angle * 152 / 17.0;
			feedback[controller_id][2] = cos(gimbal_pitch_angle);
			break;

		default:
			if (controller_types[controller_id] < 0) {
				feedback[controller_id][0] = feedback[-controller_types[controller_id]][0];
				feedback[controller_id][1] = feedback[-controller_types[controller_id]][1];
			}
			break;
	}
}

/*

		Estimation
	The goal is to build four estimates of the robots state.
	kee_state: kinematic encoder estimate, found using motor feedback speeds (filtered) and kinematics (scaled measurements)
	imu_state: integration of IMU accel + gyro (chassis + gimbal)	(integrated measurements)
	kee_imu_pos: integration of a fusion (k * a) + ((1-k) * b), w/ k <= 1. of the two velocity states (kee_state, imu_state) (integrated estimates)
	enc_odm_pos: position of the robot based on an integration of encoders and the imu mag data (independant wrt the other estimate, doesnt use same measurements) (integrated & scaled measurements)

	Using independant measurement values to build kee_state, imu_state and enc_odm_pos will help us reduce noise and improve estimates.
	kee_imu_pos is the most unreliable as it is an integrated estimate (can amplify errors in the estimate).
*/
void Controller_Manager::estimate_state(float* gimbal_imu, float dt) {

	// float imu_accel_state[2];

	// compute the kee velocity estimate
	// turn motor speed feedback to robot speed feedback
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		kee_state[i] = 0;
		for (int j = 0; j < MAX_NUM_RM_MOTORS; j++) {
			kee_state[i] += chassis_kinematics[i][j] * feedback[j][1]; // speed of motor j * forward_kinematics[i][j]
		}
	}

	kee_state[3] *= PI / (180.0 * 17.0); // pitch motor teeth and deg to rad
	kee_state[4] *= PI / (180.0 * 17.0 * 2.2); // yaw motor teeth and deg to rad

	// compute the accelerometer velocity estimate
	// turn inertial measurement to robot state measurement
	// rotate2D(chassis_imu, imu_accel_state, imu_offset_angle);

	// velocity of IMU relative to world in the chassis reference frame
	// imu_state[0] += imu_accel_state[0] * dt;
	// imu_state[1] += imu_accel_state[1] * dt;
	// imu_state[2] = chassis_imu[5] * 0.017453; // gyro yaw
	// imu_state[3] = gimbal_imu[4] * 0.017453; // pitch NEED TO DERIVATIVE FILTER ENCODERS TO FIND THIS (or use motors)
	// imu_state[4] = imu_yaw.filter((246.0 / 17.0) * gimbal_imu[5] / (3.9 * 2.2)) + 0.0115; // yaw imu est
	imu_state[4] = imu_yaw.filter(-gimbal_imu[5]); // yaw imu est
	// imu_state[5] = 0; // feeder (can leave zero)
	// imu_state[6] = 0; // constant (can leave zero)

	// fuse velocity estimates (increase k to 'add gain')
	// float weights[REMOTE_CONTROL_LEN] = {(r * chassis_imu[5]), (r * chassis_imu[5]), 0, 0, 0, 0, 0};
	// float compensated_imu_state[REMOTE_CONTROL_LEN];
	// weighted_vector_addition(imu_state, weights, 1, -1, REMOTE_CONTROL_LEN, compensated_imu_state);
	float imu_weight[REMOTE_CONTROL_LEN] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}; 
	float kee_weight[REMOTE_CONTROL_LEN] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}; 
	n_weighted_vector_addition(imu_state, kee_state, imu_weight, kee_weight, REMOTE_CONTROL_LEN, kee_imu_state);

	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		kee_imu_pos[i] += kee_imu_state[i] * dt;
	}

	// kee_imu_pos[4] = kee_imu_pos[4] / 2.0;

	// some robots have an encoder, some do not
	if (encoder_bias[0] > 1000) {
		gimbal_pitch_angle = wrap_angle(-feedback[int(encoder_bias[0] / 1000)][0] * 0.11184210526) + 0.3;
	}
	else {
		gimbal_pitch_angle = -wrap_angle(enc_filters[0].filter((encoders[0] - encoder_bias[0]) * PI / 180));
	}

	// get the encoder angles as radians
	// gimbal_yaw_angle = wrap_angle(enc_filters[1].filter((encoders[1] - encoder_bias[1]) * PI / 180));
	gimbal_yaw_angle = wrap_angle((encoders[1] - encoder_bias[1]) * PI / 180);

	
	enc_odm_pos[2] = wrap_angle(kee_imu_pos[4] - gimbal_yaw_angle);		// also uses kee + imu integration, shhhhh...
	enc_odm_pos[3] = gimbal_pitch_angle;					// puts the enc in enc_odm_pos
	enc_odm_pos[4] = wrap_angle(kee_imu_pos[4]); // + chassis_yaw;

	float odom[2] = {encoders[2], encoders[3]};
	float odom_components[2];
	odom_diff(odom, odom_prev, odom_components)

	enc_odm_pos[0] = 0;
	enc_odm_pos[1] = 0;

	// fuse position estimates (kinda pointless just use one or the other)
	// weighted_vector_addition(enc_odm_pos, kee_imu_pos, 0.8, 0.2, REMOTE_CONTROL_LEN, position_est);
}

void Controller_Manager::set_input(float* control_input) {
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		input[i] = control_input[i];
	}
}