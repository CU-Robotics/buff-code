#include <Arduino.h>
#include "buff_cpp/timing.h"
#include "motor_drivers/rm_can_interface.h"


#define TARGET_CAN_BUS 2
#define TARGET_ESC_TYPE 1
#define TARGET_ESC_ID 1

RM_CAN_Interface rm_can_ux;

int run_can_test() {
    byte test[3] = {TARGET_CAN_BUS, TARGET_ESC_TYPE, TARGET_ESC_ID};
    rm_can_ux.set_index(0, test);

    Serial.println("Reading motor 1");


    CAN_message_t tmp;

    uint32_t duration = 5000;

    timer_set(0);
    while(timer_info_ms(0) < duration) {
        if (rm_can_ux.can2.read(tmp)) {
            // Serial.println(tmp.id);
			int8_t motor_id = rm_can_ux.motor_index_from_return(1, tmp.id);
            // Serial.println(motor_id);
			rm_can_ux.set_feedback(1, &tmp);
			if (motor_id == 0) {
                Serial.println(rm_can_ux.motor_arr[0].data[0]);
				// Serial.printf("\t[%i] Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}
    }

    return 0;
}

int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};
	Serial.println("Start can read tests");
	delay(2000);

	int errors = run_can_test();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}