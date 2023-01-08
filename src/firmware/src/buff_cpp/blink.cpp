#include "buff_cpp/blink.h"

uint32_t blinker_timer_mark;
bool blinker_status;

void setup_blink() {
	// Hardware setup
	blinker_status = false;
	blinker_timer_mark = ARM_DWT_CYCCNT;

	pinMode(BLINK_PIN, OUTPUT);
	digitalWrite(BLINK_PIN, blinker_status);
}

void blink(){
	if ((1E6/F_CPU)*(ARM_DWT_CYCCNT - blinker_timer_mark) > BLINK_RATE_US){
		blinker_status = !blinker_status;
		blinker_timer_mark = ARM_DWT_CYCCNT;
		digitalWrite(BLINK_PIN, blinker_status);
	}
}