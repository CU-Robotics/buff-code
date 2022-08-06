#include <Arduino.h>

//#include "drivers/mpu6050.h"
#include "drivers/rmmotor.h"
#include "drivers/buff_hid.h"

void init_HID(HID_Device* hid){
	hid->input = HIDBuffer();
	hid->output = HIDBuffer();
	hid->imu = MPU6050();
	hid->receiver = DR16();
	//usb_init();

}

int8_t send_HID(HID_Device* hid, Motor_LUT* minfo){

	hid->imu.read(&hid->output);
	hid->receiver.read(&hid->output);

	static int motor_seek = 0;
	while (motor_seek <= 29) {
		if (minfo->motors[motor_seek].read(&hid->output) == 0){
			break;
		}
		motor_seek++;
	}
	if (motor_seek >= 29){
		motor_seek = 0;
	}

	int8_t n = usb_rawhid_send(&hid->output.data, 0);

	hid->output.reset();

	return n;
}

int8_t read_HID(HID_Device* hid, Motor_LUT* minfo){
	int i = 0; 
	hid->input.reset();
	// int n= 0;
	int8_t n = usb_rawhid_recv(&hid->input.data, 0);

	while (i < n){
		if (hid->input.seek() == 'X') {
			if (hid->input.seek() == 'X') {
				switch (hid->input.seek()) {
					case 'M':
						Serial.println("Init new motor");
						new_motor(minfo, 
							hid->input.seek(), 
							hid->input.seek(), 
							hid->input.seek(), 
							hid->input.seek());
						break;

					case 'I':
						hid->imu.init(hid->input.seek(), 
							hid->input.seek());
						break;

					case 'D':
						hid->receiver.init(int(hid->input.seek()));
						break;
					
				}
			}
		}
		if (hid->input.seek_ptr < i) {
			break;
		}

		i = hid->input.seek_ptr;
	}

	return n;
}