#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

// Uses builtin LED to show when HID is connected
Device_Manager::Device_Manager(){
	setup_blink();
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
void Device_Manager::initializer_report_handle() {
	int limit_offset;
	int controller_type;
	int controller_id = input_report.get(2);

	float limits[4];
	float gains[MOTOR_FEEDBACK_SIZE];
	byte tmp[3];

	output_report.put(1, input_report.get(1));
	output_report.put(2, input_report.get(2));

	switch (input_report.get(1))	{
		case 0:
			for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
				input_report.rgets(tmp, (3 * i) + 2, 3);
				rm_can_ux.set_index(i, tmp);
				// Serial.printf("New CAN device %i: [%i %i %i]\n", i, tmp[0], tmp[1], tmp[2]);

				if (rm_can_ux.motor_arr[i].can_bus > 0) {
					output_report.rputs(tmp, (3 * i) + 2, 3);
				}
			}

		case 1:
			// set the local controllers gains, and limits
			// Serial.printf("controller %i\ngains: ", controller_id);
			controller_type = input_report.get(3);
			for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
				gains[i] = input_report.get_float((4 * i) + 4);
				// Serial.printf("%f, ", gains[i]);
			}

			limit_offset = (4 * MOTOR_FEEDBACK_SIZE) + 4;

			for (int i = 0; i < 2; i++) {
				limits[i] = input_report.get_float((4 * i) + limit_offset);
				limits[i + 2] = input_report.get_float((4 * (i + 2)) + limit_offset);
			}
			// Serial.printf("\nlimits: %f, %f, %f, %f\n", limits[0], limits[1], limits[2], limits[3]);

			controller_manager.init_controller(controller_id, controller_type, gains, limits);
			controller_switch = 1;												// enable local control
			break;

		case 2:	// Use the assumption IK.shape = K.T.shape
			for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
				// Serial.printf("%f, ", input_report.get_float(chunk_offset + (4 * j)));
				controller_manager.chassis_inverse_kinematics[controller_id][j] = input_report.get_float(j + 3);					
				controller_manager.chassis_kinematics[j][controller_id] = input_report.get_float(REMOTE_CONTROL_LEN + j + 3);					
			}
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
void Device_Manager::feedback_request_handle() {
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
void Device_Manager::control_input_handle() {
	output_report.put(1, input_report.get(1));

	int data_offset = 3;
	int block_offset = CAN_MOTOR_BLOCK_SIZE * input_report.get(2);
	float tmp[2 * REMOTE_CONTROL_LEN];

	output_report.put(2, input_report.get(2));
	output_report.put(3, input_report.get(3));
	output_report.put(4, input_report.get(4));

	switch (input_report.get(1)) {
		case 0:
			for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE; i++) {
				// set the control from the robot "robot control"
				controller_manager.output[block_offset + i] = input_report.get_float((4 * i) + data_offset);
			}

			controller_switch = 0;												// block "local control"
			break;

		case 1:
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
					controller_manager.get_manager_report(tmp);
					output_report.put(5, controller_switch);
					break;

				default:
					break;
			}

			for (int i = 0; i < 2 * REMOTE_CONTROL_LEN; i++) {
				output_report.put_float((4 * i) + data_offset, tmp[i]);
			}

			break;

		case 2:
			if (controller_switch == 1) {	// Only user can put the bot in auto aim
				break;
			}

			// set the gimbal input from ros control
			controller_manager.input[3] = input_report.get_float(2);
			controller_manager.input[4] = input_report.get_float(6);
			controller_manager.input[5] = input_report.get_float(10);
			// Serial.printf("Reference set to %f %f %f\n", controller_manager.input[3], controller_manager.input[4], controller_manager.input[5]);
			
			controller_switch = 2;												// block local gimbal input
			break;

		case 3:
			// set the input from ros control
			controller_manager.input[0] = input_report.get_float(2);
			controller_manager.input[1] = input_report.get_float(6);
			controller_manager.input[2] = input_report.get_float(10);
			controller_manager.input[3] = input_report.get_float(14);
			controller_manager.input[4] = input_report.get_float(18);
			controller_manager.input[5] = input_report.get_float(22);
			
			controller_switch = 3;												// block local input
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
void Device_Manager::sensor_request_handle() {
	// use input_report.data[1] as the sensor to read
	// imu = 0 (36 bytes = 9 floats), dr16 = 1 (28 bytes = 7 floats)
	int sensor = input_report.get(1);
	output_report.put(1, sensor);

	switch (sensor) {
		case 0:
			for (int i = 0; i < IMU_DOF; i++){
				output_report.put_float((4 * i) + 2, chassis_imu.data[i]);
			}
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
			// configuration / initializers
			initializer_report_handle();
			// Serial.println("initializer received");
			break;

		case 1:
			// motor feedback data request
			feedback_request_handle();
			// Serial.println("Motor feedback requested");
			break;

		case 2:
			// controls reports
			control_input_handle();
			// Serial.println("Control input");
			break;

		case 3:
			// sensor data request
			sensor_request_handle();
			// Serial.println("Sensor data requested");
			break;

		default:
			break;
	}
}

/*
	Check for an HID packet

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::hid_input_switch(){
	/*
		Check if there is an HID input packet, 
		if there is one check the packet request
		and build the packet.
		@param:
		  None
		@return:
		  If a packet was read or not
	*/
	switch (input_report.read()) {
		case 64:
			blink();										// only blink when connected to a robot
			report_switch();
			output_report.put_int32(60, ARM_DWT_CYCCNT);
			break;

		case 0:
			// controller_switch = 1;
			rm_can_ux.zero_can();							// Shutdown motors if can disconnects
			break;
		
		default:
			break;
	}

	output_report.write();
	output_report.clear();
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
	rm_can_ux.write_can();

	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		rm_can_ux.read_can(i);		
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
	controller_manager.gimbal_enc[0] = yawEncoder.getAngle();
	controller_manager.gimbal_enc[1] = pitchEncoder.getAngle();

	switch (sensor_switch) {
		case 0:
			chassis_imu.read_lsm6dsox_accel();
			sensor_switch += 1;
			break;

		case 1:
			chassis_imu.read_lsm6dsox_gyro();
			sensor_switch += 1;
			break;

		case 2:
			chassis_imu.read_lis3mdl();
			sensor_switch += 1;
			break;

		case 3:
			ref.read_serial();
			controller_manager.power_buffer = ref.data.power_buffer;
			sensor_switch += 1;
			break;

		case 4:
			receiver.read();
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
	bool new_references = timer_info_us(2) >= 10;

	controller_manager.estimate_state(chassis_imu.data, chassis_imu.yaw, dt);

	if (controller_switch == 1) {								// Use DR16 control input
		controller_manager.set_input(receiver.data);
	}
	else if (controller_switch == 2) {							// Use DR16 chassis control and HID gimbal control
		controller_manager.input[0] = receiver.data[0];
		controller_manager.input[1] = receiver.data[1];
		controller_manager.input[2] = receiver.data[2];
	}
		

	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		// Serial.printf("%i output: %f\n", i, controller_manager.output[i]);
		if (new_references) {
			controller_manager.set_reference(i);			
			timer_set(2);
		}

		controller_manager.set_feedback(i, rm_can_ux.motor_arr[i].data, rm_can_ux.motor_arr[i].roll_over);
	}

	if (controller_switch != 0) {								// Use the HIDLayers CAN output values or the controllers
		controller_manager.step_motors();
	}


	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {				// Send the controllers output to the can interface
		// Serial.printf("%i output: %f\n", i, controller_manager.output[i]);
		rm_can_ux.set_output(i, controller_manager.output[i]);
	}
}
