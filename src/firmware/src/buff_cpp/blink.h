#include <Arduino.h>

#ifndef BUFF_BLINKER_H
#define BUFF_BLINKER_H

#define BLINK_RATE_US 250000
#define BLINK_PIN LED_BUILTIN

extern uint32_t blinker_timer_mark;
extern bool blinker_status;

void setup_blink();
void blink();

#endif