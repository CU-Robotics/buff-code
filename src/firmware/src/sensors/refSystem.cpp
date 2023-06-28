#include <Arduino.h>
#include <iostream>
#include "refSystem.h"

uint8_t generateCRC8(byte* data, uint32_t len) {
	uint8_t CRC8 = 0xFF;
    while (len-- > 0) {
        uint8_t curr = CRC8 ^ (*data++);
        CRC8 = CRC8Lookup[curr];
    }
    return CRC8;
}

uint16_t generateCRC16(byte* data, uint32_t len) {
	uint16_t CRC16 = 0xFFFF;
    while (len-- > 0) {
        uint8_t curr = *data++;
        CRC16 = (CRC16 >> 8) ^ CRC16Lookup[(CRC16 ^ static_cast<uint16_t>(curr)) & 0x00FF];
    }
    return CRC16;
}

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

	while (Serial2.available() > 1) {
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

			else if (cmd_id == 0x201) { //robo stat
				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				data.robot_id = temp;
				data.robot_type = data.robot_id % 100;
				if (data.robot_id < 100) data.team_color = 0;
				else data.team_color = 1;
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

				if(int(data.robot_id) == 1) {      //red hero

					if(int(data.robot_level) >= 1 && int(data.robot_level) <=3)
						data.robot_level = int(data.robot_level);

					data.red_hero_hp = temp_hp;
					data.red_hero_max_hp = temp_max_hp;
					
				}
				else if(int(data.robot_id) == 3) {    //red infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.red_infantry_hp = temp_hp;
					data.red_infantry_max_hp = temp_max_hp;
				}
				else if(int(data.robot_id) == 7) {    //red sentry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.red_sentry_hp = temp_hp;
					data.red_sentry_max_hp = temp_max_hp;
				}
				else if(int(data.robot_id) == 101) {    //blue hero

					if(int(data.robot_id) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);      

					data.blue_hero_hp = temp_hp;
					data.blue_hero_max_hp = temp_max_hp;
					
				}
				else if(int(data.robot_id) == 103) {    //blue infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						data.robot_level = int(robot_level);

					data.blue_infantry_hp = temp_hp;
					data.blue_infantry_max_hp = temp_max_hp;
					
				}
				else if(int(data.robot_id) == 107) {    //blue sentry

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
				if (temp_stat % 5 == 0 && temp_stat < 500) data.robot_1_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot_1 barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				if (temp_stat % 5 == 0 && temp_stat < 2000) data.robot_1_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // setting robot 1 speed
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				if (temp_stat <= 30.0 && temp_stat > 0.0) {
					data.robot_1_speed_lim = temp_stat;
				}

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

			else if (cmd_id == 0x301) {
				int8_t f_bytes[4] = {0};

				while (Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				temp_stat = temp_stat | (temp<<8);   
				int command = temp_stat;

				float tmp_float_1;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[3] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[2] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[1] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[0] = temp;
				memcpy(&tmp_float_1, f_bytes, 4);
				
				float tmp_float_2;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[3] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[2] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[1] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[0] = temp;
				memcpy(&tmp_float_2, f_bytes, 4);

				float tmp_float_3;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[3] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[2] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[1] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				f_bytes[0] = temp;
				memcpy(&tmp_float_3, f_bytes, 4);

				// Sentry state update
				if (command == 0x0207) {
					data.sentry_pos[0] = tmp_float_1;
					data.sentry_pos[1] = tmp_float_2;
					data.sentry_pos[2] = tmp_float_3;
				}
				// Infantry state update
				else if (command == 0x0203) {
					data.infantry_pos[0] = tmp_float_1;
					data.infantry_pos[1] = tmp_float_2;
					data.infantry_pos[2] = tmp_float_3;
				}
				// Sentry goal update
				else if (command == 0x0208) {
					Serial.println("Recieved Sentry goal update");
					data.sentry_goal[0] = tmp_float_1;
					data.sentry_goal[1] = tmp_float_2;
					data.sentry_goal[2] = tmp_float_3;
					if (data.robot_type == 7) memcpy(data.autonomy_pos, data.sentry_goal, 3);
				}
				// Infantry goal update
				else if (command == 0x0204) {
					Serial.println("Recieved Infantry goal update");
					data.infantry_goal[0] = tmp_float_1;
					data.infantry_goal[1] = tmp_float_2;
					data.infantry_goal[2] = tmp_float_3;
					if (data.robot_type == 1) memcpy(data.autonomy_pos, data.infantry_goal, 3);
				}
			}
		}
	}
	return 1;
}

void RefSystem::write_serial(float* enc_odm_pos) {
	byte msg[128] = {0};
	int msg_len = 0;

	if (data.pending_sentry_send) {
		Serial.println("Sending recall command (refSystem.cpp 412)");
		uint16_t content_id = 0x0208;
		uint16_t rec_id = 0x0007;
		rec_id = rec_id | (data.robot_id & 0xFF00);
		float d[3] = {0};
		d[0] = data.sentry_send_goal[0];
		d[1] = data.sentry_send_goal[1];
		d[2] = data.sentry_send_goal[2];
		write_update(msg, &msg_len, content_id, rec_id, d);
		if (Serial2.write(msg, msg_len)) data.pending_sentry_send = false;
		return;
	};

	uint16_t content_id = 0x0200 | data.robot_type;
	float d[3] = {0};
	if (send_sw == 0) {
		// Send state update to hero (Sentry, Infantry)
		content_id = content_id | data.robot_type;
		uint16_t hero_rec_id = 0x0001;
		hero_rec_id = hero_rec_id | (data.robot_id & 0xFF00);
		d[0] = enc_odm_pos[0];
		d[1] = enc_odm_pos[1];
		d[2] = enc_odm_pos[4];
		write_update(msg, &msg_len, content_id, hero_rec_id, d);
		send_sw++;
	} else if (send_sw == 1) {
		// Send state update to infantry (Sentry)
		content_id = content_id | data.robot_type;
		uint16_t inf_rec_id = 0x0003;
		inf_rec_id = inf_rec_id | (data.robot_id & 0xFF00);
		d[0] = enc_odm_pos[0];
		d[1] = enc_odm_pos[1];
		d[2] = enc_odm_pos[4];
		write_update(msg, &msg_len, content_id, inf_rec_id, d);
		send_sw = 0;
	}

	byte msg_graphics[128] = {0};
	int msg_graphics_len;
	// switch (graphics_sw) {
	// 	// Update primary graphics
	// 	case 0:
	// 		write_primary_graphics_update(msg_graphics, &msg_graphics_len);
	// 		graphics_sw++;
	// 		break;
	// 	case 1:
	// 		write_secondary_graphics_update(msg_graphics, &msg_graphics_len);
	// 		graphics_sw = 0;
	// 		break;
	// 	default:
	// 		graphics_sw = 0;
	// }
	write_secondary_graphics_update(msg_graphics, &msg_graphics_len);
	Serial2.write(msg_graphics, msg_graphics_len);
}

// Send an update out to another robot
void RefSystem::write_update(byte* msg, int* msg_len, uint16_t content_id, int rec_id, float* update_data) {
	// frame header
	msg[0] = 0xA5;
	msg[1] = 20;
	msg[2] = 0x00;
	msg[3] = get_seq();
	msg[4] = generateCRC8(msg, 4);

	// cmd 0x0301
	msg[5] = 0x01;
	msg[6] = 0x03;

	// content ID
	msg[7] = content_id;
	msg[8] = content_id >> 8;

	// sender ID
	msg[9] = data.robot_id;
	msg[10] = data.robot_id >> 8;

	// receiver ID
	msg[11] = rec_id;
	msg[12] = rec_id >> 8;

	for (int i = 0; i < 3; i++) {
		byte* f_bytes = (byte*)&update_data[i];
		msg[13+i*4] = f_bytes[3];
		msg[14+i*4] = f_bytes[2];
		msg[15+i*4] = f_bytes[1];
		msg[16+i*4] = f_bytes[0];
	}

	uint16_t footerCRC = generateCRC16(msg, 25);
	msg[25] = (footerCRC & 0x00FF);
	msg[26] = (footerCRC >> 8);
	
	*msg_len = 27;
}

void RefSystem::write_primary_graphics_update(byte* msg, int* msg_len) {


	// YOU ARE IN THE WRONG PLACE

	// frame header
	msg[0] = 0xA5;
	msg[1] = 20;
	msg[2] = 0x00;
	msg[3] = get_seq();
	msg[4] = generateCRC8(msg, 4);

	// cmd 0x0301
	msg[5] = 0x01;
	msg[6] = 0x03;

	// content ID
	msg[7] = 0x04; // Draw 7 graphics
	msg[8] = 0x01;

	// sender ID
	msg[9] = data.robot_id;
	msg[10] = data.robot_id >> 8;

	// reciever ID
	if (data.robot_id >> 8) {
		// blue
		msg[11] = data.robot_type+100; // DJI Moment
	} else {
		// red
		msg[11] = data.robot_type;
	}
	msg[12] = 0x01;

	// generate graphics
	byte* graphic = {0};
	uint8_t operation = graphics_init ? 2 : 1;

	//generate_graphic(graphic, "name", operation, type, num_layers, color, start_angle, end_angle, width, start_x, start_y, radius, end_x, end_y);
	for (int i = 0; i < 15; i++) msg[13+i] = graphic[i];
	// Copy lines 531-532 up to 6 more times

	uint16_t footerCRC = generateCRC16(msg, 118);
	msg[118] = (footerCRC & 0x00FF);
	msg[119] = (footerCRC >> 8);
	
	*msg_len = 119;
}

void RefSystem::write_secondary_graphics_update(byte* msg, int* msg_len) {
	// frame header
	msg[0] = 0xA5;
	msg[1] = 21; // CHANGE
	msg[2] = 0x00;
	msg[3] = get_seq();
	msg[4] = generateCRC8(msg, 4);

	// cmd 0x0301
	msg[5] = 0x01;
	msg[6] = 0x03;

	// content ID
	msg[7] = 0x01; // Draw 7 graphics // CHANGE
	msg[8] = 0x01;

	// sender ID
	//Serial.println(data.robot_id, HEX);
	msg[9] = data.robot_id;
	msg[10] = 0x00;

	// reciever ID
	/*if (data.robot_id >> 8) {
		// blue
		msg[11] = data.robot_type+100; // DJI Moment
	} else {
		// red
		msg[11] = data.robot_type;
	}*/
	msg[11] = data.robot_id;
	msg[12] = 0x01;

	// generate graphics
	byte graphic[15] = {0};
	uint8_t operation = graphics_init ? 1 : 2;
	int type = 2;
	int num_layers = 9;
	int color = 1;
	int start_angle = 0;
	int end_angle = 0;
	int width = 4;
	int start_x = 1920/2 + selector_pos[0];
	int start_y = 1080/2 + selector_pos[1];
	int radius = 10;
	int end_x = 0;
	int end_y = 0;
	generate_graphic(graphic, "rbt", operation, type, num_layers, color, start_angle, end_angle, width, start_x, start_y, radius, end_x, end_y);
	for (int i = 0; i < 15; i++) msg[13+i] = graphic[i];
	// Copy lines 531-532 up to 6 more times

	uint16_t footerCRC = generateCRC16(msg, 28); // CHANGE
	msg[28] = (footerCRC & 0x00FF); // CHANGE
	msg[29] = (footerCRC >> 8); // CHANGE
	
	*msg_len = 30;
}

void RefSystem::generate_graphic(byte* graphic, char name[3], int operation, int type, int num_layers, int color, int start_angle, int end_angle, int width, int start_x, int start_y, int radius, int end_x, int end_y) {
	graphic[0] = name[0];
	graphic[1] = name[1];
	graphic[2] = name[2];
	
	graphic[3] = (operation) | (type << 3) | (num_layers << 6); //0x49;
	graphic[4] = (num_layers >> 2) | (color << 2) | (start_angle << 6); //0x04;
	graphic[5] = (start_angle >> 2) | (end_angle << 7); //0x00;
	graphic[6] = (end_angle >> 1); //0x00;
	
	graphic[7] = (width);
	graphic[8] = (width >> 8) | (start_x << 2);
	graphic[9] = (start_x >> 6) | (start_y << 5);
	graphic[10] = (start_y >> 3);
	
	graphic[11] = (radius);
	graphic[12] = (radius >> 8) | (end_x << 2);
	graphic[13] = (end_x >> 6) | (end_y << 5);
	graphic[14] = (end_y >> 3);
	
	return graphic;
}

uint8_t RefSystem::get_seq() {
	seq++;
	return seq;
}