#include <Arduino.h>
#include "sensors/refSystem.h"

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define USER_SHUTDOWN					0
#define ROBOT_DEMO_MODE					1
#define USER_DRIVE_MODE					2
#define AUTONOMY_MODE					3
#define NO_DR16_PACKET					4
#define REMOTE_CONTROL_LEN 				7
#define JOYSTICK_X_SENSITIVITY 			900
#define JOYSTICK_Y_SENSITIVITY 			900
#define JOYSTICK_PAN_SENSITIVITY 		200
#define JOYSTICK_PITCH_SENSITIVITY 		100
#define MOUSE_SENSITIVITY 0.5
#define CHASSIS_SPEED 900.0

#define SPINRATE_STILL 900.0
#define SPINRATE_TRANSLATE 800.0
#define SPINRATE_IDLE 200.0

#define FEEDSPEED_DEFAULT 500.0

#define FLYWHEEL_SPEED 900.0

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

struct DR16 {
		DR16();
		DR16(HardwareSerial*);
		void print_receiver_input(byte*);
		void print_control_data();
		int generate_control(RefSystem ref);
		void control_test();
		int read(RefSystem ref); 	// return the user mode input (different from the control mode)

		float numBytes;
		unsigned long lastTime;
		uint32_t timestamp;

		float data[7];
		int safety_shutdown;

		bool no_path = true;

		bool beyblade_mode = 0;
		int shooter_mode = 0;
		bool sentry_control_hud = 0;

		bool shift_prev = 0;
		bool f_prev = 0;
		bool r_prev = 0;
		bool g_prev = 0;
		bool b_prev = 0;
		bool c_prev = 0;
		
		HardwareSerial* serial;
};

#endif