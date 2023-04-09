#include <Arduino.h>

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define REMOTE_CONTROL_LEN 7
#define JOYSTICK_X_SENSITIVITY 0.05
#define JOYSTICK_Y_SENSITIVITY 1.5
#define JOYSTICK_PAN_SENSITIVITY 0.01
#define JOYSTICK_PITCH_SENSITIVITY 0.01

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
			7: const
		*/

float normalize_channel(int16_t);
float wrap_radians(float);

struct DR16_DATA {
	float l_stick_x;
	float l_stick_y;
	float r_stick_x;
	float r_stick_y;
	float wheel;
	int l_switch;
	int r_switch;
};

struct DR16 {
		DR16();
		DR16(HardwareSerial*);
		void print_receiver_input(byte*);
		void print_control_data();
		void generate_control_from_joysticks();
		void generate_output();
		void control_test();
		bool read();

		float numBytes;
		unsigned long lastTime;

		float data[7];
		DR16_DATA out;

		float demo[];
		
		HardwareSerial* serial;
};

#endif