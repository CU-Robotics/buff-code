#include <Arduino.h>

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

	// unsigned long t = micros();
	hid->imu.read(&hid->output);
	// Serial.println(micros() - t);
	hid->receiver.read(&hid->output);
	read_motors(minfo, &hid->output);

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
		switch (hid->input.seek()) {
			case 'M':
				if (hid->input.seek() == 'M'){
					new_motor(minfo, 
						hid->input.seek(),
						hid->input.seek(), 
						hid->input.seek(), 
						hid->input.seek());
				}
				break;

			case 'm':
				if (hid->input.seek() == 'm'){
					int id = hid->input.seek(); 
					float power = hid->input.seek_f32();;
					
					minfo->motors[id].setPower(power);
				}

			case 'I':
				if (hid->input.seek() == 'I'){
					hid->imu.init(hid->input.seek(), hid->input.seek());
				}
				break;

			case 'D':
				if (hid->input.seek() == 'D'){
					hid->receiver.init(hid->input.seek());
					hid->input.seek();
				}
				break;

			default:
				break;
		}

		if (hid->input.seek_ptr < i) {
			break;
		}

		i = hid->input.seek_ptr;
	}

	return n;
}