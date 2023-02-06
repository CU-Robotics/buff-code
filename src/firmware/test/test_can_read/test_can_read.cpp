#include <Arduino.h>
#include "unity.h"
#include "motor_drivers/rm_can_interface.cpp"


RM_CAN_Interface rm_can_ux;

void setUp() {
	// set stuff up here
}

void tearDown() {
	// clean stuff up here
}

int run_can_test() {
    UNITY_BEGIN();

    byte test[3] = {2,1,1};
    rm_can_ux.set_index(0, test);

    Serial.println("Reading motor 1");
    CAN_message_t tmp;
    while(1) {
        if (rm_can_ux.can2.read(tmp)) {
            // Serial.println(tmp.id);
			int8_t motor_id = rm_can_ux.motor_idx_from_return(1, tmp.id);
            // Serial.println(motor_id);
			rm_can_ux.set_feedback(1, &tmp);
			if (motor_id == 0) {
                Serial.println(rm_can_ux.motor_index[0].data[0]);
				// Serial.printf("\t[%i] Found Device: %X:%i (rid, motor_id)\n", timer_info_us(0), tmp.id, motor_id);
			}
		}
    }

    return UNITY_END();
}

void setup() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	delay(2000);

	run_can_test();
}
void loop() {}