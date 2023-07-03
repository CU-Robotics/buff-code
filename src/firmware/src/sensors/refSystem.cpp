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
	graphics_init[0] = true;
	graphics_init[1] = true;
	graphics_init[2] = true;
}

bool RefSystem::read_serial() {
	byte enter_code, temp; //These 3 lines initialize all our temporary variables
	uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;  
	uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;
	uint32_t temp_launch_speed;
	//Serial.print("read_serial");
	int num_packets = 0;
	while (Serial2.available() > 1) {
		enter_code = Serial2.read();

		if(enter_code == 0xA5) { // It looks for this value that signifies that there is about to be a transmission of data
			num_packets++;
			byte header[4] = {0};
			header[0] = enter_code;
			while(Serial2.readBytes(&temp, 1) != 1) {} // Get the length of the packet
			data_length = temp;
			header[1] = temp;
			while(Serial2.readBytes(&temp, 1) != 1) {}
			data_length = data_length | (temp << 8);
			header[2] = temp;

			while(Serial2.readBytes(&temp, 1) != 1) {}
			seq = temp;
			header[3] = temp;
			while(Serial2.readBytes(&temp, 1) != 1) {}
			crc = temp;
			
			if (temp != generateCRC8(header, 4)){
				//Serial.println("Bad Packet");
				//Serial.println(cmd_id, HEX);
				continue;
			}

			while(Serial2.readBytes(&temp, 1) != 1) {}
			cmd_id = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			while(Serial2.readBytes(&temp, 1) != 1) {}
			cmd_id = cmd_id | (temp << 8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			int bytes_read = 0;
			//Serial.print("Command id: ");
			//Serial.println(cmd_id, HEX);
			if(cmd_id == 0x202) {   //power and heat data

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				data.chassis_voltage = temp_stat | (temp<<8);

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				bytes_read++;
				temp_stat = temp;
				Serial2.readBytes(&temp, 1);    // Setting chasis output current
				bytes_read++;
				data.chassis_current = temp_stat | (temp<<8);

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				bytes_read++;
				Serial2.readBytes(&temp, 1);
				bytes_read++;
				Serial2.readBytes(&temp, 1);
				bytes_read++;
				Serial2.readBytes(&temp, 1);
				bytes_read++;

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				bytes_read++;
				temp_stat = temp;
				Serial2.readBytes(&temp, 1);
				bytes_read++;
				data.power_buffer = temp_stat | (temp<<8);

				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
				}
				Serial2.readBytes(&temp, 2);
			}

			else if(cmd_id == 0x1) {  //stage 1

				// Serial.println("received cmd_id inside 1"); 

				while(Serial2.readBytes(&temp, 1) != 1) {}        //This waits till another byte of data is available
				bytes_read++;
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
				bytes_read++;
				unix_time = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				unix_time = unix_time | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.rem_time = int(unix_time);

				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
				}
				Serial2.readBytes(&temp, 2);
			}

			else if(cmd_id == 0x2) {   //results for 2

				//Serial.println("received cmd_id inside 2"); 
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				comp_stat = temp;

				if(comp_stat == 0)
					data.comp_result = 'D';
					
				else if(comp_stat == 1)
					data.comp_result = 'R';
					
				else if(comp_stat == 2)
					data.comp_result = 'B';

				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
				}
				Serial2.readBytes(&temp, 2);
			}

			else if (cmd_id == 0x201) { //robo stat
				////////////////////////////////////////////////////////////////////////////
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				data.robot_id = temp;
				data.robot_type = data.robot_id % 100;
				if (data.robot_id < 100) data.team_color = 0;
				else data.team_color = 1;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				data.robot_level = temp;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_hp = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

				////////////////////////////////////////////////////////////////////////////
				
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_max_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
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
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot _ 1 cooling value
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				if (temp_stat % 5 == 0 && temp_stat < 500) data.robot_1_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot_1 barrel heat limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				if (temp_stat % 5 == 0 && temp_stat < 2000) data.robot_1_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // setting robot 1 speed
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				if (temp_stat <= 30.0 && temp_stat > 0.0) {
					data.robot_1_speed_lim = temp_stat;
				}

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot_2 cooling value
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	// robot 2 barrel heat limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	//robot 2 speed limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_2_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} 	// 42mm cooling value
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}	// 42mm barrel heat limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {}		// robot 42mm speed limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_42_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1) {} // robot power limit
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				data.robot_power_lim = temp_stat;

				/////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				data.chassis_on = (temp_stat >> 0) & 1;
		  		data.gimbal_on = (temp_stat >> 1) & 1;
				data.shooter_on = (temp_stat >> 2) & 1;

				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
				}
				Serial2.readBytes(&temp, 2);
			}

			else if (cmd_id == 0x204) {
				Serial2.readBytes(&temp, 1);
				bytes_read++;
				data.robot_health = temp;

				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
				}
				Serial2.readBytes(&temp, 2);
            }

			else if (cmd_id == 0x301) {
				byte msg[18] = {0};
				//Serial.println(header[1]);
				//Serial.println(header[2]);
				//Serial.println(data_length);
				int8_t f_bytes[4] = {0};

				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;
				msg[0] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp_stat | (temp<<8);   
				msg[1] = temp;
				int command = temp_stat;

				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;
				msg[2] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp_stat | (temp<<8); 
				msg[3] = temp; 
				int send_id = temp_stat;

				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp;
				msg[4] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				temp_stat = temp_stat | (temp<<8); 
				msg[5] = temp; 
				int rec_id = temp_stat;

				for (int i = 0; i < 7; i++){ //Read 7 bytes cuz dji decided to slap in a good ol 0x00007856341210, you know, cuz why not
					while (Serial2.readBytes(&temp, 1) != 1) {}
					//Serial.println(temp, HEX); // In case you want to see the bs for yourself	
				}

				float tmp_float_1;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[3] = temp;
				msg[6] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[2] = temp;
				msg[7] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[1] = temp;
				msg[8] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[0] = temp;
				msg[9] = temp;
				memcpy(&tmp_float_1, f_bytes, 4);
				
				float tmp_float_2;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[3] = temp;
				msg[10] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[2] = temp;
				msg[11] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[1] = temp;
				msg[12] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[0] = temp;
				msg[13] = temp;
				memcpy(&tmp_float_2, f_bytes, 4);

				float tmp_float_3;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[3] = temp;
				msg[14] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[2] = temp;
				msg[15] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[1] = temp;
				msg[16] = temp;
				while (Serial2.readBytes(&temp, 1) != 1) {}
				bytes_read++;
				f_bytes[0] = temp;
				msg[17] = temp;
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
					data.sentry_goal[0] = tmp_float_1;
					data.sentry_goal[1] = tmp_float_2;
					data.sentry_goal[2] = tmp_float_3;
					// Serial.print("Sender:");
					// Serial.println(send_id);
					// Serial.print("Receiver:");
					// Serial.println(rec_id);
					Serial.print("x:");
					Serial.print(data.sentry_goal[0]);
					Serial.print("m, y:");
					Serial.print(data.sentry_goal[1]);
					Serial.print("m, theta:");
					Serial.print(data.sentry_goal[2]);
					Serial.println("rad");
					if (data.robot_type == 7) memcpy(data.autonomy_pos, data.sentry_goal, 3);
				}
				// Infantry goal update
				else if (command == 0x0204) {
					data.infantry_goal[0] = tmp_float_1;
					data.infantry_goal[1] = tmp_float_2;
					data.infantry_goal[2] = tmp_float_3;
					if (data.robot_type == 1) memcpy(data.autonomy_pos, data.infantry_goal, 3);
				}

				//Serial.println("I bet I made it here");
				while (bytes_read < data_length) {
					bytes_read++;
					Serial2.readBytes(&temp, 1);
					//Serial.println(temp, HEX);
				}
				//Serial.println("Probably an infinite while loop");
				Serial2.readBytes(&temp, 2);
			}
		}
	}
	//if (num_packets > 1){
		//Serial.print("num packets");
		//Serial.println(num_packets);
	//}
	//Serial2.clear();
	return 1;
}

void RefSystem::write_serial(float* enc_odm_pos) {
	byte msg[128] = {0};
	int msg_len = 0;
	if (data.pending_sentry_send) {
		Serial.println("Sending goal update");
		uint16_t content_id = 0x0208;
		uint16_t rec_id = 7;
		rec_id = 100 * data.team_color + rec_id;
		float d[3] = {0};
		d[0] = data.sentry_send_goal[0];
		d[1] = data.sentry_send_goal[1];
		d[2] = data.sentry_send_goal[2];
		//for (int i = 0; i < 3; i++){
		//	Serial.println(d[i]);
		//}
		write_update(msg, &msg_len, content_id, rec_id, d);
		for (int i = 0; i < msg_len; i++){
			//Serial.println(msg[i], HEX);
		}
		Serial.println();
		if (Serial2.write(msg, msg_len)) {
			sentry_send_counter++;
			if (sentry_send_counter >= 2){
				data.pending_sentry_send = false;
				sentry_send_counter = 0;
			}
		}
		return;
	};
	byte msg_graphics[128] = {0};
	int msg_graphics_len;
	if (field_graphics_update_pending){
		field_graphics_update_pending = false;
		write_field_graphics_update(msg_graphics, &msg_graphics_len);
		Serial2.write(msg_graphics, msg_graphics_len);
		for (int i = 0; i < msg_graphics_len; i++){
			//Serial.println(msg_graphics[i], HEX);
		}
		//Serial.println();
		return;
	}else if (primary_graphics_update_pending){
		primary_graphics_update_pending = false;
		write_primary_graphics_update(msg_graphics, &msg_graphics_len);
		Serial2.write(msg_graphics, msg_graphics_len);
		return;
	}else if (!graphics_init[2] && send_sw == 0){
		write_secondary_graphics_update(msg_graphics, &msg_graphics_len);
		Serial2.write(msg_graphics, msg_graphics_len);
		//Serial.println("a");
		if (data.robot_type == 7){
			send_sw++;
		}
	}else if (data.robot_type == 7 && send_sw == 1){ //Send sentry position to infantry
		//Serial.println("b");
		uint16_t content_id = 0x0200 | data.robot_type;
		float d[3] = {0};
		content_id = content_id | data.robot_type;
		uint16_t inf_rec_id = 0x0003;
		inf_rec_id = inf_rec_id | (data.robot_id & 0xFF00);
		d[0] = enc_odm_pos[0];
		d[1] = enc_odm_pos[1];
		d[2] = enc_odm_pos[4];
		write_update(msg, &msg_len, content_id, inf_rec_id, d);
		//Serial.println("c");
		Serial2.write(msg, msg_len);
		//Serial.println("d");
		send_sw = 0;
	}
	
	/*
	//Serial.println(send_sw);
	switch (send_sw){
		case 0:
			Serial.println("It is here");
			Serial2.write(msg_graphics, msg_graphics_len);
			send_sw++;
		case 1:
			Serial.println("Also here");
			Serial2.write(msg, msg_len);
			Serial.println("Here too");
			send_sw = 0;		
	}*/

	/*
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
	}*/
	
	
}

// Send an update out to another robot
void RefSystem::write_update(byte* msg, int* msg_len, uint16_t content_id, int rec_id, float* update_data) {
	// frame header
	int data_bytes = 12;
	msg[0] = 0xA5;
	msg[1] = 6+data_bytes;
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
		float temp = update_data[i];
		//Serial.println(temp);
		//Serial.print(f_bytes[0], HEX);
		//Serial.print(f_bytes[1], HEX);
		//Serial.print(f_bytes[2], HEX);
		//Serial.println(f_bytes[3], HEX);
		msg[13+i*4] = f_bytes[3];
		msg[14+i*4] = f_bytes[2];
		msg[15+i*4] = f_bytes[1];
		msg[16+i*4] = f_bytes[0];
	}

	uint16_t footerCRC = generateCRC16(msg, 13+data_bytes);
	msg[13+data_bytes] = (footerCRC & 0x00FF);
	msg[14+data_bytes] = (footerCRC >> 8);
	
	//for (int i = 7; i < 25; i++){
	//	Serial.println(msg[i], HEX);
	//}
	//Serial.println();
	*msg_len = 15+data_bytes;
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
	uint8_t operation;
	if (graphics_init[0]){
		graphics_init[0]= false;
		operation = 1;
	}else{
		operation = 2;
	}

	//generate_graphic(graphic, "name", operation, type, num_layers, color, start_angle, end_angle, width, start_x, start_y, radius, end_x, end_y);
	for (int i = 0; i < 15; i++) msg[13+i] = graphic[i];
	// Copy lines 531-532 up to 6 more times

	uint16_t footerCRC = generateCRC16(msg, 118);
	msg[118] = (footerCRC & 0x00FF);
	msg[119] = (footerCRC >> 8);
	
	*msg_len = 119;
}

void RefSystem::write_secondary_graphics_update(byte* msg, int* msg_len) {
	//Serial.println("Trying to print");
	int num_graphics = 5;
	// frame header
	msg[0] = 0xA5;
	msg[1] = 6+15*num_graphics; // CHANGE
	msg[2] = 0x00;
	msg[3] = get_seq();
	msg[4] = generateCRC8(msg, 4);

	// cmd 0x0301
	msg[5] = 0x01;
	msg[6] = 0x03;

	// content ID
	if (num_graphics == 1){
		msg[7] = 0x01;
	}else if (num_graphics == 2){
		msg[7] = 0x02;
	}else if (num_graphics == 5){
		msg[7] = 0x03; // Draw 7 graphics // CHANGE
	}else if (num_graphics == 7){
		msg[7] = 0x04;
	}	
	msg[8] = 0x01;

	// sender ID
	msg[9] = data.robot_id;
	msg[10] = 0x00;

	// reciever ID
	msg[11] = data.robot_id;
	msg[12] = 0x01;


	uint8_t operation;
	if (graphics_init[1]){
		graphics_init[1]= false;
		operation = 1;
	}else{
		operation = 2;
	}
	int j = 0;
	// generate graphics
	byte graphic[15] = {0};
	generate_graphic(graphic, 
		"inf", //namer
		operation, //Operation
		1, //type
		8, //num_layer
		8, //color
		0, //start_angle
		0, //end_angle
		12, //width
		(1920/2)-6+(int(data.infantry_pos[0]*50)-300)-1000*!show_map, // + temp_rts_pos[0], //start_x
		(1080/2)-6+(int(data.infantry_pos[1]*50)-200)-1000*!show_map, //+ temp_rts_pos[1], //stary_y
		0, //radius
		(1920/2)+6+(int(data.infantry_pos[0]*50)-300)-1000*!show_map, //end_x
		(1080/2)+6+(int(data.infantry_pos[1]*50)-200)-1000*!show_map); //ednd_y
	for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
	}
	j++;
	

	if (num_graphics > 1){
		generate_graphic(graphic, 
			"gol", //namer
			operation, //Operation
			2, //type
			8, //num_layer
			2, //color
			0, //start_angle
			0, //end_angle
			12, //width
			1920/2 + (temp_rts_pos[0]*50)-300-1000*!show_map, //start_x
			1080/2 + (temp_rts_pos[1]*50)-200-1000*!show_map, //stary_y
			6, //radius
			0-1000*show_map, //end_x
			0-1000*show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;		
		
	}
	
	if (num_graphics > 2){
		
		
		generate_graphic(graphic, 
			"wl1", //namer
			operation, //Operation
			0, //type
			8, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(4.675*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(2.728*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(4.675*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(5.728*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
		
		generate_graphic(graphic, 
			"rbt", //namer
			0, //Operation
			1, //type
			8, //num_layer
			5, //color
			0, //start_angle
			0, //end_angle
			6, //width
			(1920/2)-15-1000*!show_map, //start_x
			(1080/2)-15-1000*!show_map, //stary_y
			10, //radius
			(1920/2)+15-1000*!show_map, //end_x
			(1080/2)+15-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
		generate_graphic(graphic, 
			"csr", //namer
			operation, //Operation
			2, //type
			8, //num_layer
			1, //color
			0, //start_angle
			0, //end_angle
			4, //width
			1920/2 + selector_pos[0] - 1000*!show_map, //start_x
			1080/2 + selector_pos[1] - 1000*!show_map, //stary_y
			10, //radius
			0, //end_x
			0); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
	}
	
	

	uint16_t footerCRC = generateCRC16(msg, 13+15*num_graphics); // CHANGE
	msg[13+15*num_graphics] = (footerCRC & 0x00FF); // CHANGE
	msg[14+15*num_graphics] = (footerCRC >> 8); // CHANGE
	
	*msg_len = 9+6+15*num_graphics;
	//Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
}

void RefSystem::write_field_graphics_update(byte* msg, int* msg_len) {
	//Serial.println("Trying to print");
	int num_graphics = 7;
	// frame header
	msg[0] = 0xA5;
	msg[1] = 6+15*num_graphics; // CHANGE
	msg[2] = 0x00;
	msg[3] = get_seq();
	msg[4] = generateCRC8(msg, 4);

	// cmd 0x0301
	msg[5] = 0x01;
	msg[6] = 0x03;

	// content ID
	if (num_graphics == 1){
		msg[7] = 0x01;
	}else if (num_graphics == 2){
		msg[7] = 0x02;
	}else if (num_graphics == 5){
		msg[7] = 0x03; // Draw 7 graphics // CHANGE
	}else if (num_graphics == 7){
		msg[7] = 0x04;
	}	
	msg[8] = 0x01;

	// sender ID
	msg[9] = data.robot_id;
	msg[10] = 0x00;

	// reciever ID
	msg[11] = data.robot_id;
	msg[12] = 0x01;


	uint8_t operation;
	if (graphics_init[2]){
		graphics_init[2]= false;
		operation = 1;
		Serial.println("Initializing Field Graphics");
	}else{
		operation = 2;
	}
	int j = 0;
	// generate graphics
	byte graphic[15] = {0};
	generate_graphic(graphic, 
			"map", //namer
			operation, //Operation
			1, //type
			0, //num_layer
			8, //color
			0, //start_angle
			0, //end_angle
			300, //width
			(1920/2)-150-1000*!show_map, // + temp_rts_pos[0], //start_x
			(1080/2)-50-1000*!show_map, //+ temp_rts_pos[1], //stary_y
			0, //radius
			(1920/2)+150-1000*!show_map, //end_x
			(1080/2)+50-1000*!show_map); //ednd_y
	for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
	}
	j++;
	

	if (num_graphics > 1){
		generate_graphic(graphic, 
			"wl1", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(4.675*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(2.728*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(4.675*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(5.728*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
		
	}
	
	if (num_graphics > 2){
		generate_graphic(graphic, 
			"wl2", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(7.325*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(2.328*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(7.325*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(5.328*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
		
		generate_graphic(graphic, 
			"cw1", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(0*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(3.231*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(1.5*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(3.231*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;

		generate_graphic(graphic, 
			"cw2", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(3.231*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(0*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(3.231*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(1.5*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;
	}
	if (num_graphics > 5){
		generate_graphic(graphic, 
			"cw3", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(8.769*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(6.5*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(8.769*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(8*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;

		generate_graphic(graphic, 
			"cw4", //namer
			operation, //Operation
			0, //type
			0, //num_layer
			7, //color
			0, //start_angle
			0, //end_angle
			int(0.25*50), //width
			(1920/2)+(int(10.5*50)-300)-1000*!show_map, //start_x
			(1080/2)+(int(4.769*50)-200)-1000*!show_map, //stary_y
			0, //radius
			(1920/2)+(int(12*50)-300)-1000*!show_map, //end_x
			(1080/2)+(int(4.769*50)-200)-1000*!show_map); //ednd_y
		for (int i = 0; i < 15; i++){
			msg[13+15*j+i] = graphic[i];
			graphic[i] = 0;
		}
		j++;	
	}
	
	

	uint16_t footerCRC = generateCRC16(msg, 13+15*num_graphics); // CHANGE
	msg[13+15*num_graphics] = (footerCRC & 0x00FF); // CHANGE
	msg[14+15*num_graphics] = (footerCRC >> 8); // CHANGE
	
	*msg_len = 9+6+15*num_graphics;
	//Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
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