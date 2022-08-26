#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_hid.h"

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 1000;


HID_Packet input;
HID_Packet output;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

CAN_message_t can_input[3];

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	can1.begin();
    can1.setBaudRate(1000000);

    can2.begin();
    can2.setBaudRate(1000000);

    can_input[0].id = 0x200;
    can_input[1].id = 0x1FF;
    can_input[2].id = 0x2FF;

    read_HID(&input);
}

void blink(){
	static bool status = false;
	static unsigned long t = millis();

	if (millis() - t > 250){
		status = !status;
		t = millis();
	}

	digitalWrite(LED_BUILTIN, status);

}


// Runs continuously
void loop() {
	top_time = micros();

	read_HID(&input);

	switch (get_can_input(&input, can_input)){
		case 1: 
			can1.write(can_input[0]);
			can1.write(can_input[1]);
			can1.write(can_input[2]);
			break;

		case 2: 
			can2.write(can_input[0]);
			can2.write(can_input[1]);
			can2.write(can_input[2]);
			break;
	}

  	CAN_message_t tmp1;
  	CAN_message_t tmp2;
  	byte buff[24];

  	can2.read(tmp1);
  	can2.read(tmp2);

  	parse_can2_output(&output, &tmp1);
  	parse_can2_output(&output, &tmp2);
  	parse_dr16_output(&output, &buff);

	write_HID(&output);

	can1.read(tmp1);
  	can1.read(tmp2);

  	parse_can1_output(&output, &tmp1);
  	parse_can1_output(&output, &tmp2);
  	parse_mpu6050_output(&output, &buff);

	write_HID(&output);

	blink();

	if (micros() - top_time > cycle_time) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}
	while (micros() - top_time < cycle_time){}
}

