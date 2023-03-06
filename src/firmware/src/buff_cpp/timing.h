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


#define NUM_BUFF_TIMERS 10

extern uint32_t timers[NUM_BUFF_TIMERS];

void init_timers();
uint32_t duration_info(uint32_t, uint32_t);
void timer_set(int);
void timer_mark(int);
uint32_t timer_info_us(int);
uint32_t timer_info_ms(int);
void timer_wait_us(int, uint32_t);
void timer_wait_ms(int, uint32_t);

#endif