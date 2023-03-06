#include <Arduino.h>

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_MS(cycles)  ((cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  ((cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))
#define CYCLE_TIME_US 1000
#define CYCLE_TIME_MS CYCLE_TIME_US / 1000


typedef union
{
	float number;
	uint8_t bytes[4];
} FLOATBYTE_t;

// Runs once
void setup() {
	Serial.begin(1000000);

	while (!Serial) {};

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	float pi = 3.14156;

	FLOATBYTE_t float_byte;
	float_byte.number = pi;

	Serial.printf("UNION\n\t%f =\t%X\t%X\t%X\t%X\n", pi, float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);

	Serial.printf("Size: %i %i\n", sizeof(pi), sizeof(float_byte.bytes));

	char tmp[4];
	sprintf(tmp, "%f", pi);

	Serial.printf("sprintf\n\t%f = %s\n", pi, tmp);
	Serial.printf("Size: %i %i\n", sizeof(pi), strlen(tmp));

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
	blink();
}
