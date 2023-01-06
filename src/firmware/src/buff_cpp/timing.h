#include <Arduino.h>

#ifndef BUFF_TIMING
#define BUFF_TIMING

// Some generic converters
#define MS_TO_US(ms) 	(ms * 1000)
#define MS_TO_NS(ms) 	(ms * 1000000)
#define US_TO_NS(us) 	(us * 1000)
#define US_TO_MS(us) 	(us / 1000)
#define NS_TO_US(ns) 	(ns / 1000)
#define NS_TO_MS(ns) 	(ns / 1000000)

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_MS(cycles)  ((cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  ((cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))

// Get time duration from two cycle counts
#define DURATION_MS(cyccnt1, cyccnt2) (CYCLES_TO_MS(cyccnt2 - cyccnt1))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))

// timer 1
int32_t timer0 = ARM_DWT_CYCCNT;
int32_t timer1 = ARM_DWT_CYCCNT;
int32_t timer2 = ARM_DWT_CYCCNT;


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

void timer_set(int timer) {
	switch (timer) {
		case 0:
			timer0 = ARM_DWT_CYCCNT;
			break;
		case 1:
			timer1 = ARM_DWT_CYCCNT;
			break;
		case 2:
			timer2 = ARM_DWT_CYCCNT;
			break;
		default:
			break;
	}
}

void timer_mark(int timer) {
 	switch (timer) {
 		case 0:
			duration_info(timer0, ARM_DWT_CYCCNT);
			break;
		case 1:
			duration_info(timer1, ARM_DWT_CYCCNT);
			break;
		case 2:
			duration_info(timer2, ARM_DWT_CYCCNT);
			break;
		default:
			break;
	}
}

int32_t timer_info_us(int timer) {
	switch (timer) {
		case 0:
			return DURATION_US(timer0, ARM_DWT_CYCCNT);
		case 1:
			return DURATION_US(timer1, ARM_DWT_CYCCNT);
		case 2:
			return DURATION_US(timer2, ARM_DWT_CYCCNT);
		default:
			return 0;
	}
}

void timer_wait_us(int timer, uint32_t duration){
	/*
	  Helper to pause for a duration. Duration starts
	when timer_set() is called.
	@param
	  duration: (uint32_t) microseconds to wait (from when timer_set() was called)
	@return
		None
	*/
	switch (timer) {
		case 0:
			while(DURATION_US(timer0, ARM_DWT_CYCCNT) < duration) {}
			break;
		case 1:
			while(DURATION_US(timer1, ARM_DWT_CYCCNT) < duration) {}
			break;
		case 2:
			while(DURATION_US(timer2, ARM_DWT_CYCCNT) < duration) {}
			break;
		default:
			break;
	}
	
}

#endif