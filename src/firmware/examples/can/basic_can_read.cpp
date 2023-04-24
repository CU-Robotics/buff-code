#include <Arduino.h>
#include "buff_cpp/timing.h"
#include "motor_drivers/rm_can_interface.h"


#define TARGET_CAN_BUS 	2
#define TARGET_ESC_TYPE 1
#define TARGET_ESC_ID 	3

#define VERBOSITY 		2

RM_CAN_Interface rm_can_ux;

int run_can_test() {
	int errors = 0;
	CAN_message_t tmp;
	uint32_t duration = 5000;
	byte test[3] = {TARGET_CAN_BUS, TARGET_ESC_TYPE, TARGET_ESC_ID};
	
	rm_can_ux.set_index(0, test);

	if (VERBOSITY) {
		print_rm_config_struct(&rm_can_ux.motor_arr[0]);
		for (int i = 0; i < MAX_CAN_RETURN_IDS; i++) {
			Serial.printf("\t[%d]\t%i\t%i\n", i, rm_can_ux.can_motor_arr[0][i], rm_can_ux.can_motor_arr[1][i]);
		}
	}

	timer_set(0);
	while(timer_info_ms(0) < duration) {
		if (rm_can_ux.can2.read(tmp)) {
			int8_t motor_id = rm_can_ux.motor_index_from_return(TARGET_CAN_BUS, tmp.id);
			rm_can_ux.set_feedback(TARGET_CAN_BUS, &tmp);

			if (VERBOSITY && motor_id == 0) {
				// Serial.printf("\t[%i] Invalid Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id - 0x201, motor_id);
				Serial.printf("Motor feebdack[0]: %f\n", rm_can_ux.motor_arr[0].data[0]);
			}
			else if (VERBOSITY) {
				errors += 1;
				Serial.printf("\t[%i] Invalid Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id - 0x201, motor_id);
			}

			Serial.println(motor_id);
		}
	}

	return errors;
}

int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};
	if (VERBOSITY) {
	    Serial.println("Running basic CAN read tests");
	    Serial.printf("\tCAN bus (number):\t%i\n\tESC Type:\t\t%i\n\tESC ID:\t\t\t%i\n", TARGET_CAN_BUS, TARGET_ESC_TYPE, TARGET_ESC_ID);
    }

	int errors = run_can_test();

	if (VERBOSITY) {
		Serial.println("Finished tests");
		Serial.printf("%i failed\n", errors);
	}

	return 0;
}