#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_hid.h"
#include "buff_can.h"
#include "mpu6050.h"
#include "dr16.h"

//		Timing variables
unsigned long top_time;
unsigned long cycle_time = 1000;


HID_Packet input;
HID_Packet output;

MPU6050 imu;
DR16 receiver;

BuffCan buffcan;

int bus_switch = 0;

// Runs once
void setup() {
	Serial.begin(1000000);

	if (Serial)
		Serial.println("-- TEENSY SERIAL START --");

 	// Hardware setup
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

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

	buffcan.read_can1(output.data);

  	output.data[33] = 1;
  	// if (imu.read(buff)) {
  	// 	parse_mpu6050_output(&output, buff);
  	// }
}

void handle2(){

	buffcan.read_can2(output.data);

  	output.data[33] = 2;
  	// if (receiver.read(buff)) {
    // 	parse_dr16_output(&output, buff);
  	// }
}

// Runs continuously
void loop() {
	top_time = micros();

	if (read_HID(&input) > 0) {
		get_can_input(&input, buffcan.input[get_can_id(&input) - 1]);
		buffcan.write();

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

		write_HID(&output);
		blink();
		clear(&input);
		clear(&output);

	}
	else{
		buffcan.zero_can();
		buffcan.write();
	}
	
	
	if (micros() - top_time > cycle_time) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}
	while (micros() - top_time < cycle_time){}
}