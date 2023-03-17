#include "revEnc.h"

RevEnc::RevEnc(uint8_t encPin) {
	// On the Teensy 4.1 encPin can be pin 0-9, 22-25, 28,2 9, 33, 36, 37, 42-47, 48-50(dups), 51, 52-53 (dups), 54
	// Is this necessary?
	if ((encPin >= 0 && encPin <= 9) || (encPin >= 22 && encPin <= 25) || encPin == 28 || encPin == 29 || encPin == 33 || encPin == 36 || encPin == 37 || (encPin >= 42 && encPin <= 54)) {
		this->inPin = encPin;
		pinMode(this->inPin, INPUT); // Set the pin used to measure the encoder to be an input
	} else {
		this->inPin = -1;
	}

	freq.begin(this->inPin, FREQMEASUREMULTI_MARK_ONLY);
}

int RevEnc::getAngleRaw() {
	 // Burn through buffer of values in freq
	while (this->freq.available() > 1) {
		this->freq.read();
	}

	int angle = round(freq.countToNanoseconds(freq.read()) / 1000.0);
	if (angle >= 1 && angle <= 1025) {
		return angle;
	}

	return -1;
}

float RevEnc::getAngle() {
	int rawAngle = this->getAngleRaw();
	return rawAngle == -1 ? -1 : map(rawAngle, 1, 1024, 0, 36000) / 100.0;
}