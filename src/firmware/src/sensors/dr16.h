#include <Arduino.h>

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define JOYSTICK_PAN_SENSITIVITY 0.1
#define JOYSTICK_PITCH_SENSITIVITY 0.1
#define REMOTE_CONTROL_LEN 7
/*
	 Driver software for the dr16 receiver.
	Parses the serial input to a robot action.

			Remote control format (float)
			0: fire (shooter toogle)
			1: toggle (mode select)
			2: x speed
			3: y speed
			4: omega
			5: pitch
			6: yaw
		*/
struct DR16 {
		DR16();
		DR16(HardwareSerial*);
		void generate_control_from_joysticks();
		bool read();

		float data[7];
		
		HardwareSerial* serial;
};

#endif