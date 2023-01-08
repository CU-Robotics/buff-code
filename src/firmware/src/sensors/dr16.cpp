#include "dr16.h"

// 1 / 660.0 = 0.00151515 (avoids an unnecesarry division)
float normalize_channel(int16_t value){
	/*
		DR16 has some issues so we fix the stick values
		to the datasheets range [364:1684] then center
		the values around 0 (value - 1024). Then scale
		to  [-1:1].
	*/
	return (max(min(float(value), 1684.0), 364.0) - 1024.0) * 0.00151515;
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

void print_receiver_input(byte* buffer){

	Serial.println("\n\t===== DR16 Input");

	for (int i = 0; i < 18; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			Serial.print(bitRead(buffer[i], j));
		}

		Serial.print(" ");
		if ((i + 1) % 6 == 0) {
			Serial.println();
		}
	}
}

void print_control_data(float* buffer){
	Serial.println("\n\t===== DR16 Data");
	Serial.printf("\n\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n\n", 
		buffer[0], buffer[1], buffer[2], buffer[3], 
		buffer[4], buffer[5], buffer[6]);
}

DR16::DR16()
{
	serial = &Serial5;
	serial->clear();
	serial->begin(100000, SERIAL_8E1_RXINV_TXINV);
}

DR16::DR16(HardwareSerial* serial_port)
{
	serial = serial_port;
	serial->clear();
	serial->begin(100000, SERIAL_8E1_RXINV_TXINV);
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


	float r_stick_x = normalize_channel(((tmp[1] & 0x07) << 8) | tmp[0]);
	float r_stick_y = normalize_channel(((tmp[2] & 0xFC) << 5) | ((tmp[1] & 0xF8) >> 3));
	float l_stick_x = normalize_channel((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6));
	float l_stick_y = normalize_channel(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1));

	// Serial.println("\n\t===== Normalize Sticks");
	// Serial.printf("\t%i\t%i\t%i\t%i\n", 
	// 	r_stick_x, r_stick_y, l_stick_x, l_stick_y);
	// Serial.printf("\t%i\t%i\t%i\t%i\n",
	// 	((tmp[1] & 0x07) << 8) | tmp[0],
	// 	((tmp[2] & 0xFC) << 5) | ((tmp[1] & 0xF8) >> 3),
	// 	(((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6),
	// 	((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1));

	data[0] = float((tmp[5] & 0x30) >> 4);			// switch 1
	data[1] = float((tmp[5] & 0xC0) >> 6);			// switch 2
	data[2] = l_stick_x;						// X & Y velocity (read directly)
	data[3] = l_stick_y;

	if (data[0] == 1.0) {						// angular velocity (theta)
		data[4] = -4000.0;
	} 
	else if (data[0] == 2.0) {
		data[4] = 4000.0;
	} 
	else {
		data[4] = 0.0;
	}

	// wrap the angles
	// pan/tilt accumulation 
	data[5] = wrap_radians(data[5] + (r_stick_y * JOYSTICK_PITCH_SENSITIVITY));
	data[6] = wrap_radians(data[6] + (r_stick_x * JOYSTICK_PAN_SENSITIVITY));
}

bool DR16::read()
{

	// if (serial->available() < 18)
	// {
	// 	return false;
	// }
	
	// else if (serial->available() % 18 != 0)
	// {
	// 	serial->clear();
	// }

	if (serial->available() == 18)
	{
		generate_control_from_joysticks();
		return true;
	}

	serial->clear();
	return false;

}