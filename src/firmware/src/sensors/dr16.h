#include <Arduino.h>

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define REMOTE_CONTROL_LEN 6
#define JOYSTICK_X_SENSITIVITY 2
#define JOYSTICK_Y_SENSITIVITY 2
#define JOYSTICK_PAN_SENSITIVITY 50
#define JOYSTICK_PITCH_SENSITIVITY 50

/*
	 Driver software for the dr16 receiver.
	Parses the serial input to a robot action.

			Remote control format (float)
			1: x speed
			2: y speed
			3: omega
			4: dpitch
			5: dyaw
			6: dfeeder
		*/

float normalize_channel(int16_t);
float wrap_radians(float);

struct DR16 {
		DR16();
		DR16(HardwareSerial*);
		void print_receiver_input(byte*);
		void print_control_data();
		void generate_control_from_joysticks();
		bool read();

		float data[7];
		
		HardwareSerial* serial;
};

#endif