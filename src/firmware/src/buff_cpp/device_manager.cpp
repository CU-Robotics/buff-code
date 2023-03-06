#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

Device_Manager::Device_Manager(){
	setup_blink();
}

void Device_Manager::initializer_report_handle() {
	int chunk_offset;
	int limit_offset;
	int chunk_size = 4 * MOTOR_FEEDBACK_SIZE;
	int controller_id = input_report.get(2);
	int block_offset = CAN_MOTOR_BLOCK_SIZE * input_report.get(2);

	float limits[4];
	float gains[MOTOR_FEEDBACK_SIZE];

	switch (input_report.get(1))	{
		case 0:
			for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
				byte tmp[3];

				input_report.rgets(tmp, (3 * i) + 2, 3);
				rm_can_ux.set_index(i, tmp);
				// Serial.printf("New CAN device %i: [%i %i %i]\n", i, tmp[0], tmp[1], tmp[2]);

				if (rm_can_ux.motor_index[i].can_bus > 0) {
					output_report.rputs(tmp, (3 * i) + 2, 3);
				}
			}

		case 1:
			// set the local controllers gains, and limits
			// Serial.printf("controller %i\ngains: ", controller_id);
			for (int i = 0; i < MOTOR_FEEDBACK_SIZE; i++) {
				gains[i] = input_report.get_float((4 * i) + 3);
				// Serial.printf("%f, ", gains[i]);
			}

			limit_offset = (4 * MOTOR_FEEDBACK_SIZE) + 3;

			for (int i = 0; i < 2; i++) {
				limits[i] = input_report.get_float((4 * i) + limit_offset);
				limits[i + 2] = input_report.get_float((4 * (i + 2)) + limit_offset);
			}
			// Serial.printf("\nlimits: %f, %f, %f, %f\n", limits[0], limits[1], limits[2], limits[3]);

			controller_manager.init_controller(controller_id, gains, limits);
			controller_switch = 1;												// enable local control
			break;

		case 2:

			chunk_size = 4 * REMOTE_CONTROL_LEN;
			block_offset = 2 * input_report.get(2);							// kinematic matrix comes in rows of two

			// if (block_offset == 0) {
			// 	Serial.println("Chassis Invers Kinematics");
			// }
			// set the local chassis model
			for (int i = 0; i < 2; i++) { 
				// Serial.printf("%i: ", block_offset + i);
				chunk_offset = (chunk_size * i) + 3;		
				for (int j = 0; j < REMOTE_CONTROL_LEN; j++) {
					// Serial.printf("%f, ", input_report.get_float(chunk_offset + (4 * j)));
					controller_manager.chassis_inverse_kinematics[block_offset + i][j] = input_report.get_float(chunk_offset + (4 * j));					
				}
				// Serial.println();
			}

			controller_switch = 1;												// enable local control
			break;

		default:
			break;
	}
}

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

void Device_Manager::control_input_handle() {
	output_report.put(1, input_report.get(1));

	int block_offset = CAN_MOTOR_BLOCK_SIZE * input_report.get(2);
	float tmp[12];

	switch (input_report.get(1)) {
		case 0:
			for (int i = 0; i < CAN_MOTOR_BLOCK_SIZE; i++) {
				// set the control from the robot "robot control"
				controller_manager.output[block_offset + i] = input_report.get_float((4 * i) + 3);
			}

			controller_switch = 0;												// block "local control"
			break;

		case 1:
			output_report.put(2, input_report.get(2));
			output_report.put(3, input_report.get(3));
			controller_manager.get_control_report(input_report.get(2), tmp);
			controller_manager.get_control_report(input_report.get(3), &tmp[6]);
			for (int i = 0; i < 12; i++) {
				output_report.put_float((4 * i) + 4, tmp[i]);
			}
			break;

		case 2:
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

void Device_Manager::sensor_request_handle() {
	// use input_report.data[1] as the sensor to read
	// imu = 0 (36 bytes = 9 floats), dr16 = 1 (28 bytes = 7 floats)
	int sensor = input_report.get(1);
	output_report.put(1, sensor);

	switch (sensor) {
		case 0:
			for (int i = 0; i < IMU_DOF; i++){
				output_report.put_float((4 * i) + 2, imu.data[i]);
			}
			break;

		case 1:
			// DR16 data is float[7]
			for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
				output_report.put_float((4 * i) + 2, receiver.data[i]);
			}
			break;

		default:
			break;
	}
}

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

void Device_Manager::push_can(){
	/*
		Read and Write to the CAN busses.
		@param:
			None
		@return:
			None
	*/
	for (int i = 0; i < NUM_CAN_BUSES; i++) {
		rm_can_ux.read_can(i);		
	}

	rm_can_ux.write_can();
}

void Device_Manager::read_sensors() {
	switch (sensor_switch) {
		case 0:
			imu.read_lsm6dsox_accel();
			sensor_switch += 1;
			break;

		case 1:
			imu.read_lsm6dsox_gyro();
			sensor_switch += 1;
			break;

		case 2:
			imu.read_lis3mdl();
			sensor_switch += 1;
			break;

		case 3:
			receiver.read();
			sensor_switch = 0;
			break;
	}
}

void Device_Manager::step_controllers(float dt) {
	bool new_references = timer_info_us(2) >= 10;

	if (controller_switch == 1) {
		controller_manager.set_input(receiver.data);
	}
	else if (controller_switch == 2) {
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
		controller_manager.set_feedback(i, rm_can_ux.motor_index[i].data, rm_can_ux.motor_index[i].roll_over);
	}

	if (controller_switch != 0) {
		controller_manager.step_motors();
	}

	for (int i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		// Serial.printf("%i output: %f\n", i, controller_manager.output[i]);
		rm_can_ux.set_output(i, controller_manager.output[i]);
	}
}