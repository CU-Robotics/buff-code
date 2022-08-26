#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_hid.h"
#include "mpu6050.h"
#include "dr16.h"

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 1000;


HID_Packet input;
HID_Packet output;

MPU6050 imu;
DR16 receiver;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

CAN_message_t can_input[6];

int bus_switch = 0;


void disable_can(CAN_message_t* msgs) {
	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 8; j++){
			msgs[i].buf[j] = 0.0;
		}
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

	can1.begin();
    can1.setBaudRate(1000000);

    can2.begin();
    can2.setBaudRate(1000000);

    can_input[0].id = 0x200;
    can_input[1].id = 0x1FF;
    can_input[2].id = 0x2FF;
    can_input[3].id = 0x200;
    can_input[4].id = 0x1FF;
    can_input[5].id = 0x2FF;

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


void handle1(){ // double readig messages makes for better syncing ???

	CAN_message_t tmp1;
  	CAN_message_t tmp2;
  	byte buff[24];

  	can1.read(tmp1);  	
  	parse_can1_output(&output, &tmp1);
  	
  	// if (imu.read(buff)) {
  	// 	parse_mpu6050_output(&output, buff);
  	// }

  	can1.read(tmp2);
  	parse_can1_output(&output, &tmp2);

	write_HID(&output);
}

void handle2(){

	CAN_message_t tmp1;
  	CAN_message_t tmp2;
  	byte buff[18];

  	can2.read(tmp1);
  	parse_can2_output(&output, &tmp1);
  	
  	if (receiver.read(buff)) {
   		parse_dr16_output(&output, buff);
  	}

  	can2.read(tmp2);
  	parse_can2_output(&output, &tmp2);

	write_HID(&output);
}

// Runs continuously
void loop() {
	top_time = micros();

	if (read_HID(&input) > 0) {
		get_can_input(&input, can_input);
		for (int i = 0; i < 3; i++)  {
			can1.write(can_input[i]);
			can2.write(can_input[i + 3]);
		}


		switch (bus_switch){
			case 0:
				handle1();
				bus_switch = 1;
				break;

			case 1:
				handle2();
				bus_switch = 0;
				break;

			default:
				break;
		}

		clear(&output);
		clear(&input);
		blink();
	}
	else{
		disable_can(can_input);
		for (int i = 0; i < 3; i++)  {
			can1.write(can_input[i]);
			can2.write(can_input[i + 3]);
		}
	}
	
	
	if (micros() - top_time > cycle_time) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}
	while (micros() - top_time < cycle_time){}
}