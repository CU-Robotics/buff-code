#include <Arduino.h>

#ifndef BUFF_BLINKER_H
#define BUFF_BLINKER_H

#define BLINK_RATE_US 250000

uint32_t blinker_timer_mark = ARM_DWT_CYCCNT;
bool blinker_status = false;

void setup_blink() {
	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
}

void blink(){

	if ((ARM_DWT_CYCCNT - blinker_timer_mark)*1E6/F_CPU > BLINK_RATE_US){
		blinker_status = !blinker_status;
		blinker_timer_mark = ARM_DWT_CYCCNT;
		digitalWrite(LED_BUILTIN, blinker_status);
	}
}

#endif