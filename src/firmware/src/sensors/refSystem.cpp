#include <Arduino.h>
#include <iostream>
#include "refSystem.h"

/*
The methods in this code have been tested on a partial ref system, but this particular code has not been 
a full ref system. I attempted to test it on a full game mode system using the ref system simulation but, I could
not get the servers to work. There was also trouble getting this up and running since, all the documentation for this
simulation and the UI itself is in Chinese. The way in which I wrote the code was tested on part of the ref system, but it seemed to
only be sending 2-3 particular messages and I could not test the full scope of the code. I feel I will be able to more
accurately test the code when I have a fully up and running ref system. 
*/

RefSystem::RefSystem() {
	Serial2.begin(115200);
}

void RefSystem::init() {
	Serial2.begin(115200);
}

bool RefSystem::read_serial() {
	byte enter_code, temp; //These 3 lines initialize all our temporary variables
	uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;  
	uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;
	uint32_t temp_launch_speed;

	while(Serial2.available() > 1) {
		enter_code = Serial2.read();

		if(enter_code == 0xA5) { // It looks for this value that signifies that there is about to be a transmission of data

			Serial2.readBytes(&temp, 1); // Get the length of the packet
			data_length = temp;
			Serial2.readBytes(&temp, 1);
			data_length = data_length | (temp << 8);

			Serial2.readBytes(&temp, 1);
			seq = temp;
			Serial2.readBytes(&temp, 1);

			crc = temp;

			while(Serial2.readBytes(&temp, 1) != 1) {}
			cmd_id = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			while(Serial2.readBytes(&temp, 1) != 1) {}
			cmd_id = cmd_id | (temp << 8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			if(cmd_id == 0x202) {   //power and heat data
				 

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				data.chassis_voltage = temp_stat | (temp<<8);    

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				temp_stat = temp;
				Serial2.readBytes(&temp, 1);    // Setting chasis output current
				data.chassis_current = temp_stat | (temp<<8);

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				Serial2.readBytes(&temp, 1);
				Serial2.readBytes(&temp, 1);
				Serial2.readBytes(&temp, 1);

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				temp_stat = temp;
				Serial2.readBytes(&temp, 1);
				data.power_buffer = temp_stat | (temp<<8);

			}

			else if(cmd_id == 0x1) {  //stage 1

				// Serial.println("received cmd_id inside 1"); 

				while(Serial2.readBytes(&temp, 1) != 1) {}        //This waits till another byte of data is available

				comp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				comp_stat = comp_stat >> 4;

				if(comp_stat == 0)
					data.curr_stage = 'P';  //pre comp stage

				else if(comp_stat == 1)
					data.curr_stage = 'S';   //Setup

				else if(comp_stat == 2)
					data.curr_stage = 'I';    //Init stage

				else if(comp_stat == 3)
					data.curr_stage = 'F';   //5 sec countdown
				
				else if(comp_stat == 4)
					data.curr_stage = 'C';   //In combat

				else if(comp_stat == 5)
					data.curr_stage = 'R';   //calc comp results

				while(Serial2.readBytes(&temp, 1) != 1) {}
				unix_time = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}
				unix_time = unix_time | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.rem_time = int(unix_time);
									
			}

			else if(cmd_id == 0x2) {   //results for 2

				//Serial.println("received cmd_id inside 2"); 
				while(Serial2.readBytes(&temp, 1) != 1) {}
				comp_stat = temp;

				if(comp_stat == 0)
					data.comp_result = 'D';
					
				else if(comp_stat == 1)
					data.comp_result = 'R';
					
				else if(comp_stat == 2)
					data.comp_result = 'B';

			}

			else if(cmd_id == 0x201) { //robo stat

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				data.robot_id = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				data.robot_level = temp;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_hp = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

				////////////////////////////////////////////////////////////////////////////
				
				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_max_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_max_hp = temp_max_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

				////////////////////////////////////////////////////////////////////////////

				// //This part looks for the robot id and then assigns values that follow depending on what robot id is transmitted

				if(int(robo_id) == 1) {      //red hero

					if(int(data.robot_level) >= 1 && int(data.robot_level) <=3)
						data.robot_level = int(data.robot_level);

					data.red_hero_hp = temp_hp;
					data.red_hero_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 3) {    //red infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.red_infantry_hp = temp_hp;
					data.red_infantry_max_hp = temp_max_hp;
				}
				else if(int(robo_id) == 7) {    //red sentry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.red_sentry_hp = temp_hp;
					data.red_sentry_max_hp = temp_max_hp;
				}
				else if(int(robo_id) == 101) {    //blue hero

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);      

					data.blue_hero_hp = temp_hp;
					data.blue_hero_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 103) {    //blue infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.blue_infantry_hp = temp_hp;
					data.blue_infantry_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 107) {    //blue sentry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.blue_sentry_hp = temp_hp;
					data.blue_sentry_max_hp = temp_max_hp;
					
				}

				////////////////////////////////////////////////////////////////////////////
			
				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot _ 1 cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_1_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot_1 barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_1_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // setting robot 1 speed
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_1_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot_2 cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	// robot 2 barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	//robot 2 speed limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	// 42mm cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}	// 42mm barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}		// robot 42mm speed limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot power limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_power_lim = temp_stat;

				/////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				data.chassis_on = (temp_stat >> 0) & 1;
		  		data.gimbal_on = (temp_stat >> 1) & 1;
				data.shooter_on = (temp_stat >> 2) & 1;
			}

			else if (cmd_id == 0x204) {
				Serial2.readBytes(&temp, 1);
				data.robot_health = temp;
            }
		}
	}
	return 1;
}

void RefSystem::write_serial() {
	byte msg[119];
	//msg = generate_hud_msg();
	// switch (data.robot_id) {
	// 	case 1:

	// }
	Serial2.write(msg, 119);
}

byte* RefSystem::generate_hud_msg() {
	byte msg[119];
	msg[0] = 0x01;
	msg[1] = 0x01;
	msg[2] = 0x00;
	msg[3] = data.robot_id;
	msg[4] = 0x00;
	msg[5] = data.robot_id;
	msg[6] = 0x00;
	msg[7] = 0x00;
	msg[8] = 0x01;
	msg[9] = 0b01000100;
	msg[10] = 0b10010000;
	msg[16] = 0xFF;
	msg[18] = 0xFF;
}

// Generates a movemement command for the Sentry
byte* RefSystem::generate_movement_command_msg() {
	byte msg[119];
	msg[0] = 0x02;
	msg[1] = retrieve_message_id();
	msg[2] = 0x00;
	msg[3] = data.robot_id;
	msg[4] = 0x00;
	msg[5] = 0x07; // Sentry
	msg[6] = 2;
	// msg[20] = ((byte*)&f)[0]
}

byte* RefSystem::generate_graphic(int identifier, int operation, int type, int num_layers, int color, int start_angle, int end_angle, int width, int x_start, int y_start, int fontsize_or_radius, int x_end, int y_end) {
	byte msg[15];

	msg[0] = 0x00;
	msg[1] = identifier & 0xff;
	msg[2] = (identifier >> (8)) & 0xff;

	msg[3] = (operation << 5) | (type << 2) | (num_layers >> 2);
	msg[4] = (num_layers << 6) | (color << 2) | (start_angle >> 6);
}

uint8_t RefSystem::retrieve_message_id() {
	message_id++;
	return message_id;
}