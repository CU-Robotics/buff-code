#include <Arduino.h>

#ifndef BUFF_DR16_H
#define BUFF_DR16_H

#define JOYSTICK_PAN_SCALE 1 / pow(2, 15)
#define JOYSTICK_PITCH_SCALE 1 / pow(2, 15)

/*
	 Driver software for the dr16 receiver.
	Parses the serial input to a robot action.

			Remote control format (int16_t)
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
		void read();

		int16_t data[7];
		
		HardwareSerial* serial;
};

#endif