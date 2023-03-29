#include <FreqMeasureMulti.h>

#ifndef RevEnc_H
#define RevEnc_H

#define MAX_REV_ENCODERS 4

#define GIMBAL_YAW_ENCODER_PIN 	 	4
#define GIMBAL_PITCH_ENCODER_PIN 	3
#define ODOM_X_ENCODER_PIN 			2
#define ODOM_Y_ENCODER_PIN 			1

class RevEnc {
	private:
		uint8_t inPin;
		FreqMeasureMulti freq;
	public:
		RevEnc(uint8_t encPin);
		int getAngleRaw();
		float getAngle();
};

#endif