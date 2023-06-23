#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
#include "buff_cpp/device_manager.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us * 1E-6;

unsigned long prev_time;


Device_Manager device_manager;						// all firmware pipelines are implemented in this object.
													// Device Manager provides a single function call for each of the pipelines
													// with unit tests we can analyze the execution time and complexity of each
													// pipeline. Then organize them into the master loop accordingly

// Runs once
void setup() {
	Serial.begin(1000000);							// the serial monitor is actually always active (for debug use Serial.println & tycmd)

	if (Serial) {
		Serial.println("\n\n-- TEENSY SERIAL START --\n\n");
		Serial.println("\033[1;33m                             .^~^.                                    ");
		Serial.println("                     .:^~7JY5PPPPP5?!:                                ");
		Serial.println("               .~!?Y5PPPP5J?!^:.:~?5BG555Y~                           ");
		Serial.println("             :?PG5Y?7~^..          .!!!!7PG5!.                        ");
		Serial.println("            ^GGY:                        .!5G5!.                      ");
		Serial.println("            ^GGJ^~~~~!^                     ~5GP7.                    ");
		Serial.println("             !PGGGGGGP?                       ~YGP7^^::....           ");
		Serial.println("            JPP5Y?!^:.                          ~YPPPPPPPPP555YYJ?~   ");
		Serial.println("           ~GG!        .~?JYYJ?~: ^JJ~     ~JJ~   ...::^^~~~!!77JGGJ  ");
		Serial.println("           JGP.      :JPGPYJJYPG! !GG7     7GG7                 7GG~  ");
		Serial.println("          .PGJ      :PGP~.    .:  !GG7     !GG7  ~?77!!~^^~???7?PG?   ");
		Serial.println("   :!?Y55Y5GG^      JGG~          !GG7     !GG7  !YYY55PPPPGGGGGGY:   ");
		Serial.println(" ^YGPY?7!7J5J       ?GG!          !GG?     7GG7        ...::::^!YGP?. ");
		Serial.println("~PGJ.               .YGG?^.  .^!  :PGP!:.:!PGP:                  ^5GY.");
		Serial.println("JGP.                  !YGGP55PGG7  :?PGGPGGPJ:                    ^GG7");
		Serial.println("?GP:         .~~:       :~!77!~:     .^~~~^.                      .PGY");
		Serial.println(":PGJ         !GGP5J7^.                                            !GG!");
		Serial.println(" ~PGJ.      .YG5~7J5PP5J!^.                                   .:!YGP7 ");
		Serial.println("  ^YGP?~^^~75G5^    :~7JGG5.                            .:!?Y5PPPY7:  ");
		Serial.println("    ^?5PPPP5J~.        .PGJ                :7Y5YJ7!^:~?YPGPY?!~^.     ");
		Serial.println("       ....            .5GY             ^75GPJ!7JY5PPP5J!^.           ");
		Serial.println("                        !GG!           ~GGY~.      .:.                ");
		Serial.println("                         ?GP~          YG5.                           ");
		Serial.println("                          ?GG?:      :JGP~                            ");
		Serial.println("                           ^YGGY?77?YGGJ:                             ");
		Serial.println("                             :!?YYYJ?!:                               \033[0m");
		Serial.println("\n\033[1;92mFW Ver. 0.9.0");
		Serial.print("Boot timehash: 0x");
		unsigned long time = micros();
		Serial.print(time & 0xFFFF, HEX);
		Serial.println("\033[0m\n");
	}
}

// Master loop
int main() {											// Basically a schudeling algorithm
	setup();
	prev_time = micros();

	while(1) {
		timer_set(0);

		// Calculate dt
		unsigned long curr_time = micros();
		float dt = (curr_time - prev_time) / 1000000.0;
		prev_time = curr_time;

		// handle any hid input output
		device_manager.read_sensors();					// read a single sensor each call (increments the sensor id automatically)		
		device_manager.step_controllers(dt);	// given the current inputs and feedback compute a control	
		device_manager.hid_input_switch(cycle_time_us);	// check for an input packet (data request/control input) handle accordingly
		device_manager.push_can();						// push data on and off the can bus

		// if (timer_info_ms(0) > cycle_time_ms) {
		// 	Serial.println("Teensy overcycled");
		// }
		timer_wait_us(0, cycle_time_us);				// normalize master loop cycle time to cycle_time_u
		// blink();										// helpful if you think the loop is crashing (light will pause)
	}
	
	return 0;
}
