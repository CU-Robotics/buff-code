#include "buff_cpp/timing.h"

uint32_t timers[NUM_BUFF_TIMERS];

void init_timers() {
	for (size_t i = 0; i < NUM_BUFF_TIMERS; i++) {
		timers[i] = ARM_DWT_CYCCNT;
	}
}

uint32_t duration_info(uint32_t start, uint32_t stop){
	/*
	  Helper to print info about *very small* durations in
	time. Prints the cycles and time in ns.
	@param
	  start: (uint32_t) value of ARM_DWT_CYCCNT at the beginning of a duration
	  stop: (uint32_t) value of ARM_DWT_CYCNT at the end of a duration
	@return
	  delta_ns: (uint32_t) duration in nanoseconds
	*/
	uint32_t delta_cycles = stop - start;
	uint32_t delta_ns = CYCLES_TO_NS(delta_cycles); 
	Serial.printf( "\t%1lu cycles, %1lu ns\n", delta_cycles, delta_ns);
	return delta_ns;
}

void timer_set(int idx) {
	timers[idx] = ARM_DWT_CYCCNT;
}

void timer_mark(int idx) {
	duration_info(timers[idx], ARM_DWT_CYCCNT);
}

uint32_t timer_info_us(int idx) {
	return DURATION_US(timers[idx], ARM_DWT_CYCCNT);
}

uint32_t timer_info_ms(int idx) {
	return DURATION_MS(timers[idx], ARM_DWT_CYCCNT);
}

void timer_wait_us(int idx, uint32_t duration){
	/*
	  Helper to pause for a duration. Duration starts
	when timer_set() is called.
	@param
	  duration: (uint32_t) microseconds to wait (from when timer_set() was called)
	@return
		None
	*/
	while(DURATION_US(timers[idx], ARM_DWT_CYCCNT) < duration) {}
}

void timer_wait_ms(int idx, uint32_t duration){
	/*
	  Helper to pause for a duration. Duration starts
	when timer_set() is called.
	@param
	  duration: (uint32_t) milliseconds to wait (from when timer_set() was called)
	@return
		None
	*/
	while(DURATION_MS(timers[idx], ARM_DWT_CYCCNT) < duration) {}
}