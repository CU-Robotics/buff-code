#include <Arduino.h>
#include "buff_lsm6dsox.h"

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_MS(cycles)  ((cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  ((cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))
#define CYCLE_TIME_US 1000
#define CYCLE_TIME_MS CYCLE_TIME_US / 1000

int imu_dev_cnt = 0;
int32_t imu_timer = ARM_DWT_CYCCNT;

Buff_LSM6DSOX imu;


void imu_read() {
	if (DURATION_US(imu_timer, ARM_DWT_CYCCNT) < 2000) {
		return;
	}

	switch (imu_dev_cnt) {
		case 0:
			imu.read_lsm6dsox_accel();
			imu_dev_cnt += 1;
			break;

		case 1:
			imu.read_lsm6dsox_gyro();
			imu_dev_cnt += 1;
			break;

		case 2:
			imu.read_lis3mdl();
			imu_dev_cnt = 0;
			break;
	}

}

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
}

void blink(){
	static bool status = false;
	static uint32_t mark = ARM_DWT_CYCCNT;

	if (CYCLES_TO_MS(ARM_DWT_CYCCNT - mark) > 250){
		status = !status;
		mark = ARM_DWT_CYCCNT;
	}

	digitalWrite(LED_BUILTIN, status);
}

// Runs continuously
void loop() {

	int32_t loop_start = ARM_DWT_CYCCNT;

	imu_read();

	
	int32_t delta_us = DURATION_US(loop_start, ARM_DWT_CYCCNT);
	if (delta_us < CYCLE_TIME_US) {
		delayMicroseconds(CYCLE_TIME_US - delta_us);
	}

	Serial.printf("Read time: %i\n", delta_us);
	imu.pretty_print_data();
}
