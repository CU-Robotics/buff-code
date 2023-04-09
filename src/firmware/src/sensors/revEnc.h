#include <FreqMeasureMulti.h>

#ifndef RevEnc_H
#define RevEnc_H

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