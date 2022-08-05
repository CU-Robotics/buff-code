#include <Arduino.h>

//#include "drivers/mpu6050.h"
#include "drivers/rmmotor.h"
#include "drivers/buff_hid.h"

void init_HID(HID_Device* hid){
	hid->input = HIDBuffer();
	hid->output = HIDBuffer();
}

int send_HID(HID_Device* hid, Motor_LUT* minfo){

	hid->imu.read(&hid->output);
	hid->receiver.read(&hid->output);

	static int motor_seek = 0;
	while (minfo->motors[motor_seek].read(&hid->output)) {
		motor_seek++;
		if (motor_seek > 29) {
			motor_seek = 0;
		}
	}

	int n = RawHID.send(hid->output.data, 0);

	hid->output.reset();

	return n;
}

int read_HID(HID_Device* hid, Motor_LUT* minfo){
	int i = 0; 
	hid->input.reset();
	int n = RawHID.recv(hid->input.data, 0);

	while (i < n - 2){
		if (hid->input.seek() == 'X') {
			if (hid->input.seek() == 'X') {
				int id;
				unsigned int canid;
				unsigned int byte_num;
				unsigned int motor_type;
				switch (hid->input.seek()) {
					case 'M':
						id = hid->input.seek();
						canid = hid->input.seek();
						byte_num = hid->input.seek();
						motor_type = hid->input.seek();
						new_motor(minfo, id, canid, byte_num, motor_type);
						break;

					case 'I':
						hid->imu = MPU6050(hid->input.seek(), hid->input.seek());
						break;

					case 'D':
						hid->receiver.init(int(hid->input.seek()));
						break;
					
				}
			}
		}
		i = hid->input.seek_ptr;
	}
	return n;
}