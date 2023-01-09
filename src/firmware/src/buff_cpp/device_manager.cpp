#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

Device_Manager::Device_Manager(){
	setup_blink();
	imu = new LSM6DSOX();
	receiver = new DR16();
	rm_can_ux = new RM_CAN_Interface();
	input_report = new Hid_Report();
	output_report = new Hid_Report();
}

Device_Manager::Device_Manager(LSM6DSOX* lsm6dsox, DR16* dr16, RM_CAN_Interface* can){
	setup_blink();
	imu = lsm6dsox;
	receiver = dr16;
	rm_can_ux = can;
	input_report = new Hid_Report();
	output_report = new Hid_Report();
}

void Device_Manager::initializer_report_handle() {
	// Fill init device handlers
	for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		byte tmp[3];
		input_report->rgets(tmp, (3 * i) + 1, 3);
		rm_can_ux->set_index(i, tmp);

		if (rm_can_ux->motor_index[i].can_bus > 0) {
			output_report->rputs(tmp, (3 * i) + 1, 3);
		}
	}
}

void Device_Manager::feedback_request_handle() {
	// use input_report.data[1] as the block ID
	// 16 motors = 4 blocks, each block is 4 motors
	// worth of feedback
	float tmp[12];
	output_report->put(1, input_report->get(1));
	rm_can_ux->get_block_feedback(input_report->get(1), tmp);
	for (int i = 0; i < 12; i++) {
		output_report->put_float((4 * i) + 2, tmp[i]);
	}
}

void Device_Manager::sensor_request_handle() {
	// use input_report.data[1] as the sensor to read
	// imu = 0 (36 bytes = 9 floats), dr16 = 1 (28 bytes = 7 floats)
	int sensor = input_report->get(1);
	output_report->put(1, sensor);
	switch (sensor) {
		case 0:
			for (int i = 0; i < IMU_DOF; i++){
				output_report->put_float((4 * i) + 2, imu->data[i]);
			}
			break;

		case 1:
			// DR16 data is int16_t[7] (38:58)
			for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
				output_report->put_float((2 * i) + 2, receiver->data[i]);
			}
			break;

		default:
			break;
	}
}

void Device_Manager::report_switch() {
	// reply to the configuration
	output_report->put(0, input_report->get(0));

	switch (input_report->get(0)) {
		case 255:
			// configuration / initializers
			initializer_report_handle();
			break;

		case 1:
			// motor feedback data request
			feedback_request_handle();
			break;

		case 2:
			// read something
			// write something
			break;

		case 3:
			// sensor data request
			sensor_request_handle();
			break;

		default:
			// On intial connection the HID packet is
			// set to init_mode
			output_report->clear();
			break;
	}
}

bool Device_Manager::hid_input_switch(){
	/*
		Check if there is an HID input packet, 
		if there is one check the packet request
		and build the packet.
		@param:
		  None
		@return:
		  If a packet was read or not
	*/
	switch (input_report->read()) {
		case 64:
			blink();
			report_switch();
			output_report->put_int32(60, timer_info_us(2));
			timer_set(2);
			break;

		default:
			output_report->put(0, 0);
			output_report->put_int32(60, 0);
			return false;
  }

  output_report->write();
  return true;
}

void Device_Manager::push_can(){
	/*
		Read and Write to the CAN busses.
		@param:
			None
		@return:
			None
	*/
	rm_can_ux->read_can1();
	rm_can_ux->read_can2();
	rm_can_ux->write_can();
}

// void sensor_read() {
//   if (DURATION_US(device_timer, ARM_DWT_CYCCNT) >= DEVICE_READ_RATE) {
//     switch (device_switch) {
//       case 0:
//         imu.read_lsm6dsox_accel();
//         device_switch += 1;
//         break;

//       case 1:
//         imu.read_lsm6dsox_gyro();
//         device_switch += 1;
//         break;

//       case 2:
//         imu.read_lis3mdl();
//         device_switch += 1;
//         break;

//       case 3:
//         receiver.read();
//         device_switch = 0;
//         break;
//     }
//   }
// }
