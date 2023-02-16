#include "dr16.h"

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

void DR16::generate_control_from_joysticks() {
	byte tmp[18];
	serial->readBytes(tmp, 18);

	// debugging (bitwise view)
	// print_receiver_input(tmp);

	// Normalize the joystick values
	float r_stick_x = bounded_map(((tmp[1] & 0x07) << 8) | tmp[0], 364, 1684, -660, 660);
	float r_stick_y = bounded_map(((tmp[2] & 0xFC) << 5) | ((tmp[1] & 0xF8) >> 3), 364, 1684, -660, 660);
	// Set the left stick to [-1:1]
	// 1 / 660.0 = 0.00151515 (avoids an unnecesarry division)
	float l_stick_x = bounded_map((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6), 364, 1684, -660, 660);
	float l_stick_y = bounded_map(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1), 364, 1684, -660, 660);

	// Serial.println("\n\t===== Normalized Sticks");
	// Serial.printf("\t%f\t%f\t%f\t%f\n", 
	// 	r_stick_x, r_stick_y, l_stick_x, l_stick_y);

	// data[0] = (tmp[5] & 0x30) >> 4;				// switch 1
	// data[1] = (tmp[5] & 0xC0) >> 6;				// switch 2
	data[0] = l_stick_x * JOYSTICK_X_SENSITIVITY;						// X & Y velocity (read directly)
	data[1] = l_stick_y * JOYSTICK_Y_SENSITIVITY;

	if ((tmp[5] & 0x30) >> 4 == 1.0) {						// angular velocity (theta)
		data[2] = -1000.0;
	} 
	else if ((tmp[5] & 0x30) >> 4 == 2.0) {
		data[2] = 1000.0;
	} 
	else {
		data[2] = 0.0;
	}

	// wrap the angles
	// pan/tilt accumulation 
	data[3] = r_stick_y * JOYSTICK_PITCH_SENSITIVITY;
	data[4] = r_stick_x * JOYSTICK_PAN_SENSITIVITY;

	if ((tmp[5] & 0xC0) >> 6 == 1.0) {						// feeder speed
		data[5] = -900.0;
	} 
	else if ((tmp[5] & 0xC0) >> 6 == 2.0) {
		data[5] = 900.0;
	} 
	else {
		data[5] = 0.0;
	}
}

void DR16::generate_output() {
	byte tmp[18];
	serial->readBytes(tmp, 18);

	float r_stick_x = bounded_map(((tmp[1] & 0x07) << 8) | tmp[0], 364, 1684, -1000, 1000) / 1000.0;
	float r_stick_y = bounded_map(((tmp[2] & 0xFC) << 5) | ((tmp[1] & 0xF8) >> 3), 364, 1684, -1000, 1000) / 1000.0;

	float l_stick_x = bounded_map((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6), 364, 1684, -1000, 1000) / 1000.0;
	float l_stick_y = bounded_map(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1), 364, 1684, -1000, 1000) / 1000.0;

	data[0] = l_stick_x;
	data[1] = l_stick_y;
	data[2] = r_stick_x;
	data[3] = r_stick_y;
	data[5] = (tmp[5] & 0x30) >> 4;				// switch 1
	data[6] = (tmp[5] & 0xC0) >> 6;				// switch 2
}

void DR16::control_test() {
	byte tmp[18];
	serial->readBytes(tmp, 18);
	for (int i = 0; i < sizeof(tmp); i++) Serial.printf("\n "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(tmp[i]));
	Serial.println();
}

bool DR16::read() {
	generate_output();
	return true;
	// if (serial->available() == 18) {
	// 	generate_output();
	// 	return true;
	// }
	// serial->clear();
	// return false;
}