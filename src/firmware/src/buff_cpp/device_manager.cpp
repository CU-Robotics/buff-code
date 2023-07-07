#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"
#include "buff_cpp/controllers.h"

// Uses builtin LED to show when HID is connected
Device_Manager::Device_Manager(){
	setup_blink();
	timer_set(2);
	timer_set(3);
	controller_switch = -1;
	gimbal_imu.init(GIMBAL_IMU_ADDR);
	// chassis_imu.init(CHASSIS_IMU_ADDR);
	ref.init();
}

/*
	There are a set of initializer reports required by drivers 
	to configure the devices. This is the case handler
	for each of those.

	TODO:
		make this more generic so there is not
	a new case for each driver.

	Author: Mitchell Scott
*/
void Device_Manager::initializer_report_handle() {	// 255
	int limit_offset;
	int controller_id = input_report.get(2);
	int controller_type = input_report.get(3);

	float limits[4];
	float gains[MOTOR_FEEDBACK_SIZE];
	byte tmp[3];

	Serial.println("initialize");

	output_report.put(1, input_report.get(1));
	output_report.put(2, input_report.get(2));

	switch (input_report.get(1))	{
		case 0:
			for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
				input_report.rgets(tmp, (3 * i) + 2, 3);
				rm_can_ux.set_index(i, tmp);
				// Serial.printf("New CAN device %i: [%i %i %i]\n", i, tmp[0], tmp[1], tmp[2]);
			}

		case 1:
			// set the local controllers gains, and limits
			// Serial.printf("controller %i\ngains: ", controller_id);

			for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
				gains[i] = input_report.get_float((4 * i) + 8);
				// Serial.printf("%f, ", gains[i]);
			}

			limit_offset = (4 * MOTOR_FEEDBACK_SIZE) + 8;

			for (int i = 0; i < 2; i++) {
				limits[i] = input_report.get_float((4 * i) + limit_offset);
				limits[i + 2] = input_report.get_float((4 * (i + 2)) + limit_offset);
			}
			if (controller_type > 200) {
				controller_type -= 256;
			}
			// Serial.printf("\nlimits %i: %f, %f, %f, %f\n", controller_id, limits[0], limits[1], limits[2], limits[3]);
			// Serial.printf("%i %i\n", controller_id, controller_type);
			// Serial.printf("%f %f %f\n", gains[0], gains[1], gains[2]);
			controller_manager.init_controller(controller_id, controller_type, gains, limits, input_report.get_float(4));
			// controller_switch = 1;												// enable local control
			break;

		case 2:	// Use the assumption IK.shape = K.T.shape
			for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
				// Serial.printf("%f, ", input_report.get_float(chunk_offset + (4 * j)));
				controller_manager.chassis_inverse_kinematics[controller_id][j] = input_report.get_float((4 * j) + 3);					
				controller_manager.chassis_kinematics[j][controller_id] = input_report.get_float(4 * (REMOTE_CONTROL_LEN + j) + 3);
				// Serial.printf("%f ", controller_manager.chassis_inverse_kinematics[controller_id][j]);	
			}
			// Serial.println();
			break;

		default:
			break;
	}
}

/*
	The device manager will continously read from can devices.
	This data is put in the DM Store. The HIDLayer
	will request these samples. This is the handler for
	that.

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::feedback_request_handle() {	// 1
	// use input_report.data[1] as the block ID
	// sensors are expected to have a 
	float tmp[12];
	output_report.put(1, input_report.get(1));

	rm_can_ux.get_block_feedback(input_report.get(1), tmp);

	// Serial.printf("Block ");
	for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE * MOTOR_FEEDBACK_SIZE; i++) {
		// Serial.printf("%f, ", tmp[i]);
		output_report.put_float((4 * i) + 2, tmp[i]);
	}
	// Serial.println();
}

/*
	The device manager will continously update control outputs. 
	The HIDLayer will request reports of this. This is the handler for
	that.

	TODO:
		Add more general data on the controller
		i.e. dumb the control mode, the input and
		timestamps. Currently this only share motor controller
		info. Talk with Mitchell about designing the reports.

	Author: Mitchell Scott
*/
void Device_Manager::control_input_handle() { // 2
	output_report.put(1, input_report.get(1));

	int data_offset = 3;
	int block_offset = CAN_MOTOR_BLOCK_SIZE * input_report.get(2);
	float tmp[2 * REMOTE_CONTROL_LEN];

	output_report.put(2, input_report.get(2));
	output_report.put(3, input_report.get(3));
	output_report.put(4, input_report.get(4));

	// Serial.printf("init %i\n", input_report.get(0))

	switch (input_report.get(1)) {
		// case 0:
		// 	if (abs(controller_switch) == 1) {	// Only user can put the bot in auto aim
		// 		break;
		// 	}
		// 	for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE; i++) {
		// 		// set the control from the robot "robot control"
		// 		controller_manager.output[block_offset + i] = input_report.get_float((4 * i) + data_offset);
		// 	}

		// 	controller_switch = 0;												// block "local control"
		// 	break;

		case 1:	// data requests (don't turn off when motors are off)
			switch (input_report.get(2)) {
				case 0:
					data_offset = 5;
					controller_manager.get_control_report(input_report.get(3), tmp);
					controller_manager.get_control_report(input_report.get(4), &tmp[6]);
					break;

				case 1:
					controller_manager.get_vel_est_report(tmp);
					break;

				case 2:
					controller_manager.get_pos_est_report(tmp);
					break;

				case 3:
					data_offset = 4;
					output_report.put(3, controller_switch);
					controller_manager.get_manager_report(tmp);
					break;

				default:
					break;
			}

			for (int i = 0; i < 2 * REMOTE_CONTROL_LEN; i++) {
				output_report.put_float((4 * i) + data_offset, tmp[i]);
			}

			break;

		case 2: //  waypoint report
			for (int i = 0; i < ROBOT_WAYPOINTS; i+=2){
				robot_waypoints[i] = input_report.get_float((4 * i) + 2);
				robot_waypoints[i+1] = input_report.get_float((4 * (i +  1)) + 2);
				// Serial.printf("Waypoint received: <%f, %f>\n", robot_waypoints[i], robot_waypoints[i+1]);
			}
			// timestamp (only if added to rosmessage when publishing)
			// timestamp = input_report.get_float((2 * sizeof(float) * ROBOT_WAYPOINTS) + 2);
			
			break;

		case 3: // autonomy control report
			// if (abs(controller_switch) == 1) {	// Only user can put the bot in autonomous mode
			// 	break;
			// }

			// set the input from ros control
			controller_manager.autonomy_input[0] = input_report.get_float(2);
			controller_manager.autonomy_input[1] = input_report.get_float(6);
			controller_manager.autonomy_input[2] = input_report.get_float(10);
			controller_manager.autonomy_input[3] = input_report.get_float(14);
			controller_manager.autonomy_input[4] = input_report.get_float(18);
			controller_manager.autonomy_input[5] = input_report.get_float(22);
			controller_manager.autonomy_input[6] = input_report.get_float(26);
			last_autonomy_read = millis();

			break;

		case 4: // position override report
			// Do not be afraid of buff_rust, it might not be readable but uses a similar function
			// signature to build the packets see buff_hid.rs:HidROS::new() and buff_hid.rs:HidROS::pipeline()
			controller_manager.enc_odm_pos[0] = input_report.get_float(2); // x pos
			controller_manager.enc_odm_pos[1] = input_report.get_float(6); // y pos
			// float yaw_est_err = input_report.get_float(10) - wrap_angle(kee_imu_pos[4]);
			if (ref.data.curr_stage != 'C') {
				float reloc_yaw_err = wrap_angle(input_report.get_float(10) - wrap_angle(controller_manager.kee_imu_pos[4]));
				controller_manager.kee_imu_pos[4] += reloc_yaw_err; // gimbal heading
				controller_manager.global_yaw_reference = controller_manager.kee_imu_pos[4];
			}
			Serial.println("Position Override");
			// timestamp (same as waypoint report)

			break;

		default:
			break;
	}
}

/*
	The device manager will continously read from each of
	the sensors. This data is put in the DM Store. The HIDLayer
	will request each of these samples. This is the handler for
	that.

	TODO:
		Add other IMU

	Author: Mitchell Scott
*/
void Device_Manager::sensor_request_handle() {	// 	3
	// use input_report.data[1] as the sensor to read
	// imu = 0 (36 bytes = 9 floats), dr16 = 1 (28 bytes = 7 floats)
	int sensor = input_report.get(1);
	output_report.put(1, sensor);

	switch (sensor) {
		case 0:
			// for (int i = 0; i < ICM20649_DOF; i++){
			// 	output_report.put_float((4 * i) + 2, chassis_imu.data[i]);
			// }
			break;

		case 1:
			// for (int i = 0; i < IMU_DOF; i++){
			// 	output_report.put_float((4 * i) + 2, gimbal_imu.data[i]);
			// }
			break;

		case 2:
			// DR16 data is float[7]
			for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
				output_report.put_float((4 * i) + 2, receiver.data[i]);
			}
			break;

		case 3:
			// REF 
			
			break;

		case 4:
			// RevEncoders
			for (int i = 0; i < MAX_REV_ENCODERS; i++) {
				// Serial.printf("%i %i %i %f %f\n", i, (4 * (2 * i)) + 2, (4 * ((2 * i) + 1)) + 2, input_report.get_float((4 * (2 * i)) + 2), input_report.get_float((4 * ((2 * i) + 1)) + 2));
				controller_manager.encoder_bias[i] = input_report.get_float((4 * (2 * i)) + 2);
				controller_manager.enc_filters[i].set_gain(input_report.get_float((4 * ((2 * i) + 1)) + 2));
			} 
			// controller_manager.encoder_bias[sensor] = 
			
			break;

		default:
			break;
	}
}

/*
	Input reports will have an ID indicating
	how to handle the report. Do this here

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::report_switch() {
	// reply to the report with the same id
	output_report.put(0, input_report.get(0));

	switch (input_report.get(0)) {
		case 255:
			Serial.println("Remote initalization");
			// configuration / initializers
			initializer_report_handle();
			lifetime = 0;
			break;

		case 1:
			// motor feedback data request
			feedback_request_handle();
			// Serial.println("Motor feedback requested");
			break;

		case 2:
			// controls reports
			control_input_handle();
			// Serial.println("Control input handle");
			break;

		case 3:
			// sensor data request
			sensor_request_handle();
			// Serial.println("Sensor data requested");
			break;

		case 4:
			// reset teensy
			SCB_AIRCR = 0x05FA0004;

		default:
			break;
	}
}

/*
	Check for an HID packet

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::hid_input_switch(uint32_t cycle_time_us){
	/*
		Check if there is an HID input packet, 
		if there is one check the packet request
		and build the packet.
		@param:
		  None
		@return:
		  If a packet was read or not
	*/
	// assure good timing of hid packets
	// timer_wait_us(3, cycle_time_us);
	lifetime += cycle_time_us / 1e6;

	switch (input_report.read()) {
		case 64:
			// Serial.printf("report %f\n", lifetime);

			blink();										// only blink when connected to hid
			report_switch();
			output_report.put_float(60, lifetime);
			output_report.write();
			output_report.clear();
			// timer_set(3);
			break;

		default:
			// Serial.println("No report");
			break;
	}
}

/*
	Push all output values to the CAN bus.
	Read all feedback messages from the CAN bus.


	TODO:
		Motor ID 4 is having issue, 
		Issac is leading the investigation.

	Author: Mitchell Scott
*/
void Device_Manager::push_can(){
	if (receiver.safety_shutdown) {
		rm_can_ux.zero_can();
		
	}
	rm_can_ux.write_can();

	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		rm_can_ux.read_can(i + 1);
	}
}

/*
	Read one of the I2c sensors and any other quick
	to read devices.


	TODO:
		- Add high range IMU driver to state machine
		- Add rev encoder to be read each loop (if fast enough)

	Author: Mitchell Scott
*/
void Device_Manager::read_sensors() {
	if (micros() - prev_ref_read_micros > 5) {
		prev_ref_read_micros = micros();
		ref.read_serial();
		controller_manager.team_color = ref.data.team_color;
		controller_manager.projectile_speed = ref.data.robot_1_speed_lim - 0.5;
		controller_manager.power_buffer = ref.data.power_buffer;
	}
	//Serial.print("1st");
	//Serial.println(Serial2.available());
	if (micros() - prev_ref_write_micros > 50000) { // Send data at 30Hz
	 	prev_ref_write_micros = micros();
	 	ref.write_serial(controller_manager.enc_odm_pos);
	}
	switch (sensor_switch) {
		case 0:
			sensor_switch += 1;
			break;

		case 1:
			sensor_switch += 1;
			controller_manager.encoders[0] = pitchEncoder.getAngle();
			controller_manager.encoders[1] = yawEncoder.getAngle();
			controller_manager.encoders[2] = xOdometryEncoder.getAngle();
			controller_manager.encoders[3] = yOdometryEncoder.getAngle();
			break;

		case 2:
			gimbal_imu.read_gyro();
			sensor_switch += 1;
			break;

		case 3:
			sensor_switch += 1;
			break;

		case 4:
			switch (receiver.read(&ref)) {
				case USER_SHUTDOWN:
					controller_switch = -1;
					break;

				case ROBOT_DEMO_MODE:
				case USER_DRIVE_MODE:
					controller_switch = 1;
					safety_counter = 0;
					break;

				case AUTONOMY_MODE:
					controller_switch = 2;
					safety_counter = 0;
					break;

				default:
					break;
			}
			sensor_switch = 0;
			break;

		default:
			sensor_switch = 0;
			break;
	}
}

/*
	Based on the control switch and the various
	input/feedback in the DM store, compute the
	motor output values = [-1:1].

	controller_switch tells the controller where input
	is coming from. This will need to be relayed to HID.
	Do this from the controller report handler and HIDLayer


	TODO:
		Add different controllers,
			- Gravity compensated
			- Power limited

		These two can inherit or replace
		the feedback controller object. The first two
		feedback values should remain unchanged (position, velocity)
		The third will be dependant on the controller type (maybe handled by each controller)
			power = i * V = v * F (current * Voltage/velocity * torque/rotational velocity * force)
			grav compensation = mglsin(theta), assume params have been identified
			and come from an initializer report.

		We can disscuss removing the velocity reference, this will make the velocity feedack
		a very strong damper i.e. Kd * (Rv - v) -> Kd * v (any non zero v will cause an opposing control force)
		(currently is used to help track velocity, should always have small gain ~< 1 due to noise)

	Author: Mitchell Scott
*/
void Device_Manager::step_controllers(float dt) {
	//Serial.println(dt * 1000);
	//Serial.println(dt * 1000);
	static int prev_shutdown = 1;

	controller_manager.estimate_state(gimbal_imu.data, dt);
	float input_buffer[REMOTE_CONTROL_LEN];

	// If we haven't recieved an autonomy input in a while, set to all zeros
	if (millis() - last_autonomy_read > 1000) for (int i = 0; i < 7; i++) controller_manager.autonomy_input[i] = 0;

	float ysc_gain = 0.0; // 0.8
	float ypc_gain = 0.0; //-150
	float ppc_gain = 0.0; //50

	float pitch_autonomy_p = 0.0;
	float yaw_autonomy_p = 0.0;
	float yaw_autonomy_d = 0.0;

	if (ref.data.robot_type == 1) {
		ysc_gain = -1.0; // 0.8
		ypc_gain = -350.0; //-150
		ppc_gain = 0.0; //50

		pitch_autonomy_p = 150;
		yaw_autonomy_p = -80;
		yaw_autonomy_d = -0.2;
	} else if (ref.data.robot_type == 3) {
		ysc_gain = -1.0; // 0.8
		ypc_gain = -350.0; //-150
		ppc_gain = 0.0; //50

		pitch_autonomy_p = 150;
		yaw_autonomy_p = -80;
		yaw_autonomy_d = -0.2;
	} else if (ref.data.robot_type == 5) {
		ysc_gain = -1.0; // 0.8
		ypc_gain = -350.0; //-150
		ppc_gain = 0.0; //50

		pitch_autonomy_p = 150;
		yaw_autonomy_p = -80;
		yaw_autonomy_d = -0.2;
	} else if (ref.data.robot_type == 7) {
		ysc_gain = -1.0; // 0.8
		ypc_gain = -350.0; //-150
		ppc_gain = 0.0; //50

		pitch_autonomy_p = 150;
		yaw_autonomy_p = -80;
		yaw_autonomy_d = -0.2;
	}

	float pitch_autonomy_err = wrap_angle((controller_manager.autonomy_input[3] - controller_manager.enc_odm_pos[3]));
	float pitch_autonomy_speed = pitch_autonomy_p * wrap_angle((controller_manager.autonomy_input[3] - controller_manager.enc_odm_pos[3]));

	float yaw_autonomy_err = wrap_angle((wrap_angle(controller_manager.autonomy_input[4]) - controller_manager.enc_odm_pos[4]));
	float yaw_autonomy_speed = yaw_autonomy_p * yaw_autonomy_err; //80 (163 Pu)
	yaw_autonomy_speed += yaw_autonomy_d * (yaw_autonomy_err - prev_yaw_autonomy_err) / dt; //0.5

	prev_yaw_autonomy_err = yaw_autonomy_err;
	//Serial.println(controller_manager.kee_imu_pos[4]);

	if (!receiver.safety_shutdown) {
		controller_manager.imu_calibrated = false;
		memcpy(input_buffer, receiver.data, REMOTE_CONTROL_LEN * sizeof(float));
		yaw_reference_buffer[yaw_reference_buffer_len-1] = input_buffer[4];
		for (int b = 0; b < yaw_reference_buffer_len-1; b++) yaw_reference_buffer[b] = yaw_reference_buffer[b+1];

		// AUTOAIM
		if (controller_switch > 1) {
			// Sentry
			if (ref.data.robot_type == 7) {
				input_buffer[3] = 150 * (-controller_manager.enc_odm_pos[3]); // Pitch towards 0
				input_buffer[4] = 0;
			}

			// Track with gimbal if we are instructed to
			if (controller_manager.autonomy_input[5] == 1 || controller_manager.autonomy_input[6] > 0) {
				//yaw_autonomy_speed += -0.3 * (yaw_autonomy_err * dt); //i term
				pitch_autonomy_speed += 1 * (pitch_autonomy_err * dt); //i term
				input_buffer[3] = pitch_autonomy_speed;
				input_buffer[4] = yaw_autonomy_speed;
				yaw_reference_buffer[0] = yaw_autonomy_speed;

				// Sentry: fire if we are looking at the target
				if (ref.data.robot_type == 7) {
					if (wrap_angle((wrap_angle(controller_manager.autonomy_input[4]) - controller_manager.enc_odm_pos[4])) > 0.3 || !controller_manager.autonomy_input[5]) { // Within 0.3rad on either side
						input_buffer[5] = 0;
					}
				}
			} else if (ref.data.robot_type == 7) {
				input_buffer[5] = 0;
			}
		}

		// AUTO MOVEMENT
		controller_manager.autonomy_goal[0] = ref.data.autonomy_pos[0];
		controller_manager.autonomy_goal[1] = ref.data.autonomy_pos[1];
		controller_manager.autonomy_goal[4] = ref.data.autonomy_pos[2];

		if ((ref.data.robot_type == 7 || ref.data.robot_type == 3) && controller_manager.autonomy_input[6] > 0 && !receiver.no_path) {
			float angle_to_target = atan2((controller_manager.autonomy_input[1] - controller_manager.enc_odm_pos[1]), (controller_manager.autonomy_input[0] - controller_manager.enc_odm_pos[0]));
			if (controller_manager.autonomy_input[6] == 2) {
				float dist = sqrt(sq(controller_manager.autonomy_input[1] - controller_manager.enc_odm_pos[1]) + sq(controller_manager.autonomy_input[0] - controller_manager.enc_odm_pos[0]));
				input_buffer[0] = 1000.0 * dist * cos(controller_manager.enc_odm_pos[4] - angle_to_target);
				input_buffer[1] = 1000.0 * dist * sin(controller_manager.enc_odm_pos[4] - angle_to_target);
			} else {
				input_buffer[0] = controller_manager.autonomy_input[2] * cos(controller_manager.enc_odm_pos[4] - angle_to_target);
				input_buffer[1] = controller_manager.autonomy_input[2] * sin(controller_manager.enc_odm_pos[4] - angle_to_target);
			}
			if (controller_manager.autonomy_input[5]) {
				input_buffer[4] = yaw_autonomy_speed;
				yaw_reference_buffer[0] = yaw_autonomy_speed;
			}
		}

		controller_manager.global_yaw_reference -= (yaw_reference_buffer[0]*17/246.0) * dt;

		float yaw_speed = -2*input_buffer[4] - (controller_manager.imu_state[4]*246/17.0);
		input_buffer[4] += ysc_gain * yaw_speed;
		float yaw_ang_err = controller_manager.global_yaw_reference - controller_manager.kee_imu_pos[4];
		input_buffer[4] += ypc_gain * yaw_ang_err;

		if (millis() - last_autonomy_read > 1000 && ref.data.robot_type == 7 && ref.data.curr_stage == 'C') input_buffer[2] = 900;

		controller_manager.set_input(input_buffer);
	} else {
		// Reset world-relative state when in safety mode
		if (safety_counter >= 500) {
			controller_manager.enc_odm_pos[0] = 0;
			controller_manager.enc_odm_pos[1] = 0;
			controller_manager.enc_odm_pos[4] = 0;
			controller_manager.kee_imu_pos[4] = 0;
			controller_manager.global_yaw_reference = controller_manager.kee_imu_pos[4];
		}
		safety_counter++;
	}

	// Reset gimbal integrator when robot is dead
	if (!ref.data.gimbal_on) {
		controller_manager.global_yaw_reference = controller_manager.kee_imu_pos[4];
	}

	// Don't use odom positionintegration on robots without the hardware for it
	if (ref.data.robot_type == 1 || ref.data.robot_type == 5) {
		controller_manager.enc_odm_pos[0] = 0;
		controller_manager.enc_odm_pos[1] = 0;
	}

	bool new_reference = timer_info_ms(2) >= 10;
	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		if (receiver.safety_shutdown == 0) {
			if (new_reference) {	// only update motor references when safety switch is off
				// Serial.printf("%f, %f, %f\n", receiver.data[4],  controller_manager.imu_state[4], yaw_speed_err);
				controller_manager.set_reference(i);			
			}
			if (prev_shutdown == 1 && i != 4 && i != 5) {
				controller_manager.biases[i] = 0; // when turning off safety mode we want to rebias the motors
			}
		}

		// setting feedback with bias = 0 will reset the reference and bias to the current motor pos
		controller_manager.set_feedback(i, rm_can_ux.motor_arr[i].data, rm_can_ux.motor_arr[i].roll_over);
	}
	if (new_reference) {
		timer_set(2);
	}

	// does not write to the motors, only produces an output value
	// this value will not be passed to CAN if safety switch is on
	// we want to generate the values anyways so we can see if they
	// are unreasonable without running the motors (debug feature) // dumb the reference is not generated in safety mode
	controller_manager.step_motors();

	// now use the safety switch to stop from setting output
	if (receiver.safety_shutdown == 0) {
		for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {				// Send the controllers output to the can interface
			// Serial.printf("%i output: %f\n", i, controller_manager.output[i]);
			rm_can_ux.set_output(i, controller_manager.output[i]);
		}
	}
	prev_shutdown = receiver.safety_shutdown;
}
