#include <Arduino.h>

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define USER_SHUTDOWN					0
#define ROBOT_DEMO_MODE					1
#define USER_DRIVE_MODE					2
#define AUTONOMY_MODE					3
#define NO_DR16_PACKET					4
#define REMOTE_CONTROL_LEN 				7
#define JOYSTICK_X_SENSITIVITY 			1.5
#define JOYSTICK_Y_SENSITIVITY 			1.5
#define JOYSTICK_PAN_SENSITIVITY 		0.3
#define JOYSTICK_PITCH_SENSITIVITY 		0.15
#define FLYWHEEL_SPEED					900

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
		int generate_control_from_joysticks();
		void generate_output();
		void control_test();
		int read(); 	// return the user mode input (different from the control mode)

		float numBytes;
		unsigned long lastTime;
		uint32_t timestamp;

		float data[7];
		DR16_DATA out;

		float demo[];
		
		HardwareSerial* serial;
};

#endif