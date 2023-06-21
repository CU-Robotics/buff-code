#include "dr16.h"
#include "buff_cpp/timing.h"
#include "sensors/refSystem.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
	(byte & 0x80 ? '1' : '0'), \
	(byte & 0x40 ? '1' : '0'), \
	(byte & 0x20 ? '1' : '0'), \
	(byte & 0x10 ? '1' : '0'), \
	(byte & 0x08 ? '1' : '0'), \
	(byte & 0x04 ? '1' : '0'), \
	(byte & 0x02 ? '1' : '0'), \
	(byte & 0x01 ? '1' : '0')


float bounded_map(int value, int in_low, int in_high, int out_low, int out_high){
	/*
		 This is derived from sthe arduino map() function.
	*/
	value = max(min(value, in_high), in_low);
	return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

float wrap_radians(float value) {
	if (value > PI) {
		return value - (2 * PI);
	}
	else if (value < -PI) {
		return value + (2 * PI);
	}

	return value;
}

DR16::DR16()
{
	serial = &Serial5;
	serial->clear();
	serial->begin(100000, SERIAL_8E1_RXINV_TXINV);
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		data[i] = 0;
	}
}

DR16::DR16(HardwareSerial* serial_port)
{
	timer_set(1);
	serial = serial_port;
	serial->clear();
	serial->begin(100000, SERIAL_8E1_RXINV_TXINV);
	for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
		data[i] = 0;
	}
}

void DR16::print_receiver_input(byte* buffer){

	Serial.println("\n\t===== DR16 Input");

	for (int i = 0; i < REMOTE_CONTROL_LEN; i++)
	{
		Serial.print(buffer[i], BIN);

		Serial.print(" ");
		if ((i + 1) % 6 == 0) {
			Serial.println();
		}
	}
}

void DR16::print_control_data(){
	//Serial.println("\n\t===== DR16 Data");
	Serial.printf("\n\t%f\t%f\t%f\t%f\t%f\t%f", 
		data[0], data[1], data[2], data[3], 
		data[4], data[5]);
}

/*
	 DR16 raw data format
		0:  [ch0.7,   ch0.6,   ch0.5,   ch0.4,   ch0.3,   ch0.2,   ch0.1,   ch0.0]
		1:  [ch1.4,   ch1.3,   ch1.2,   ch1.1,   ch1.0,  ch0.10,   ch0.9,   ch0.8]
		2:  [ch2.1,   ch2.0,  ch1.10,   ch1.9,   ch1.8,   ch1.7,   ch1.6,   ch1.5]
		3:  [ch2.9,   ch2.8,   ch2.7,   ch2.6,   ch2.5,   ch2.4,   ch2.3,   ch2.2]
		4:  [ch3.6,   ch3.5,   ch3.4,   ch3.3,   ch3.2,   ch3.1,   ch3.0,  ch2.10]
		5:  [s2H,       s2L,     s1H,     s1L,  ch3.10,   ch3.9,   ch3.8,   ch3.7]
		6:  [                              mouse_x_L                             ]
		7:  [                              mouse_x_H                             ]
		8:  [                              mouse_y_L                             ]
		9:  [                              mouse_y_H                             ]
		10: [                              mouse_z_L                             ]
		11: [                              mouse_z_H                             ]
		12: [lmb,       lmb,     lmb,     lmb,     lmb,     lmb,     lmb,     lmb]
		13: [rmb,       rmb,     rmb,     rmb,     rmb,     rmb,     rmb,     rmb]
		14: [key,       key,     key,     key,     key,     key,     key,     key]
		15: [key,       key,     key,     key,     key,     key,     key,     key]
		16: [                              reserved                              ]
		17: [                                text                                ]
	*/
	/*
		Remote control format (int16_t)
		0: fire (shooter toogle)
		1: toggle (mode select)
		2: x speed
		3: y speed
		4: omega
		5: pitch
		6: yaw
	*/

int DR16::generate_control(RefData ref_data) {
	byte tmp[18];
	serial->readBytes(tmp, 18);

	// debugging (bitwise view)
	// print_receiver_input(tmp);

	// Normalized Controller Inputs
	float r_stick_x = bounded_map(((tmp[1] & 0x07) << 8) | tmp[0], 364, 1684, -1000, 1000) / 1000.0;
	float r_stick_y = bounded_map(((tmp[2] & 0x3F) << 5) | ((tmp[1] & 0xF8) >> 3), 364, 1684, -1000, 1000) / 1000.0;
	float l_stick_x = bounded_map((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6), 364, 1684, -1000, 1000) / 1000.0;
	float l_stick_y = bounded_map(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1), 364, 1684, -1000, 1000) / 1000.0;
	float wheel = bounded_map((tmp[17] << 8) | tmp[16], 364, 1684, -1000, 1000) / 1000.0;
	int l_switch = (tmp[5] & 0xC0) >> 6;
	int r_switch = (tmp[5] & 0x30) >> 4;

	// Keyboard and Mouse Inputs
	bool key_w = tmp[14] & 0b00000001;
	bool key_s = tmp[14] & 0b00000010;
	bool key_a = tmp[14] & 0b00000100;
	bool key_d = tmp[14] & 0b00001000;
	bool key_shift = tmp[14] & 0b00010000;
	bool key_ctrl = tmp[14] & 0b00100000;
	bool key_q = tmp[14] & 0b01000000;
	bool key_e = tmp[14] & 0b10000000;
	bool key_r = tmp[15] & 0b00000001;
	bool key_f = tmp[15] & 0b00000010;
	bool key_g = tmp[15] & 0b00000100;
	bool key_z = tmp[15] & 0b00001000;
	bool key_x = tmp[15] & 0b00010000;
	bool key_c = tmp[15] & 0b00100000;
	bool key_v = tmp[15] & 0b01000000;
	bool key_b = tmp[15] & 0b10000000;

	int mouse_x = (int16_t)(tmp[6] | (tmp[7] << 8));
	int mouse_y = (int16_t)(tmp[8] | (tmp[9] << 8));
	int mouse_z = (int16_t)(tmp[10] | (tmp[11] << 8));
	bool l_mouse_button = tmp[12];
	bool r_mouse_button = tmp[13];

	float feedrate_bps_continuous = 8;
	float feedrate_bps_burst = 16;
	// Infantry, Standard, and Sentry
	if (ref_data.robot_type == 3 || ref_data.robot_type == 5 || ref_data.robot_type == 7 && ref_data.robot_1_cool_val != -1) {
		feedrate_bps_continuous = ref_data.robot_1_cool_val/10.0;
		feedrate_bps_burst = (ref_data.robot_1_cool_val+ref_data.robot_1_barr_heat_lim)/10.0;
		if (ref_data.robot_type != 5 && feedrate_bps_burst > 10) {
			feedrate_bps_burst = 10;
		} else if (ref_data.robot_type == 5 && feedrate_bps_burst > 20) {
			feedrate_bps_burst = 20;
		}
	}

	// Safety Switch
	safety_shutdown = 0;
	// GAME MODE
	if (l_switch == 2.0) {
		// Sentry -- Fully autonomous
		if (ref_data.robot_type == 7) {
			data[2] = SPINRATE_IDLE;
			data[6] = FLYWHEEL_SPEED;
		} 
		// Infantry, Standard, and Hero -- User driving
		else {
			// Toggle Sentry Control HUD
			if (key_f && !f_prev) sentry_control_hud = !sentry_control_hud;
			f_prev = key_f;

			// Send autonomy command
			if (key_r && !r_prev) {
				// TODO: Send autonomy command
			}
			r_prev = key_r;

			// Chassis Translation
			data[0] = (key_d - key_a) * CHASSIS_SPEED;
			data[1] = (key_w - key_s) * CHASSIS_SPEED;

			// Chassis Spin
			if (key_ctrl && !ctrl_prev) beyblade_mode = !beyblade_mode;
			ctrl_prev = key_ctrl;
			if (beyblade_mode || key_shift) {
				if (key_w || key_s || key_d || key_a) data[2] = SPINRATE_TRANSLATE; // Spin slower while translating
				else data[2] = SPINRATE_STILL; // Spin fast when still
			} else {
				data[2] = (key_x - key_z) * SPINRATE_TRANSLATE + SPINRATE_IDLE;
			}

			// Gimbal
			if (!sentry_control_hud) {
				data[3] = -mouse_y * MOUSE_SENSITIVITY;
				data[4] = mouse_x * MOUSE_SENSITIVITY;
			} else {
				data[3] = 0;
				data[4] = 0;
			}

			// Shooter/Feeder
			if (key_q) shooter_mode = 0;
			if (key_e) shooter_mode = 1;

			if (l_mouse_button && !sentry_control_hud) {
				switch (shooter_mode) {
					case 0:
						if (ref_data.robot_type == 3 || ref_data.robot_type == 7) data[5] = feedrate_bps_continuous * 45.24;
						else if (ref_data.robot_type == 5) data[5] = feedrate_bps_continuous * 28.27;
						else data[5] = FEEDSPEED_DEFAULT;
						break;
					case 1:
						if (ref_data.robot_type == 3 || ref_data.robot_type == 7) data[5] = feedrate_bps_burst * 45.24;
						else if (ref_data.robot_type == 5) data[5] = feedrate_bps_burst * 28.27;
						else data[5] = FEEDSPEED_DEFAULT;
						break;
					default:
						data[5] = FEEDSPEED_DEFAULT;
				}
			} else data[5] = 0;

			data[6] = FLYWHEEL_SPEED; // Always keep the flywheel on

			if (r_mouse_button) return AUTONOMY_MODE; // Engage autonomous gimbal when right mouse button is pressed
			else return USER_DRIVE_MODE;
		}
	// DEMO MODE
	} else if (l_switch == 3.0) {
		// Chassis translation and spin
		data[0] = l_stick_x * JOYSTICK_X_SENSITIVITY;
		data[1] = l_stick_y * JOYSTICK_Y_SENSITIVITY;
		data[2] = SPINRATE_STILL * wheel;

		// Gimbal
		data[3] = r_stick_y * JOYSTICK_PITCH_SENSITIVITY;
		data[4] = r_stick_x * JOYSTICK_PAN_SENSITIVITY;

		// Shooter/Feeder
		if (r_switch == 1.0) {
			if (ref_data.robot_type == 3 || ref_data.robot_type == 7) data[5] = feedrate_bps_continuous * 45.24;
			else if (ref_data.robot_type == 5) data[5] = feedrate_bps_continuous * 28.27;
			else data[5] = FEEDSPEED_DEFAULT;
			data[6] = FLYWHEEL_SPEED;
		} else if (r_switch == 3.0) {
			data[5] = 0.0;
			data[6] = FLYWHEEL_SPEED;
		} else {
			data[5] = 0.0;
			data[6] = 0.0;
		}

		if (r_stick_y < -0.95) {
			data[3] = 0;
			return AUTONOMY_MODE;
		}
		else return ROBOT_DEMO_MODE;
	} else { // Engage SAFETY mode when the switch is in position 1. Also acts as the default.
		for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
			data[i] = 0;
		}
		safety_shutdown = 1;
		return USER_SHUTDOWN;
	}
}

int DR16::read(RefData ref_data) {
	if (serial->available() == 18) {
		timer_set(1);
		return generate_control(ref_data);
	}

	if (serial->available() > 18) {
		serial->clear();
	}

	if (timer_info_ms(1) > 200) {
		for (int i = 0; i < REMOTE_CONTROL_LEN; i++) {
			data[i] = 0;
		}
		
		safety_shutdown = 1;
		return USER_SHUTDOWN;
	}
	else {
		return NO_DR16_PACKET;
	}
}