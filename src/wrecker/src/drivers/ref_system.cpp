#include <Arduino.h>
#include <iostream>
#include "ref_system.h"

/*
The methods in this code have been tested on a partial ref system, but this particular code has not been 
a full ref system. I attempted to test it on a full game mode system using the ref system simulation but, I could
not get the servers to work. There was also trouble getting this up and running since, all the documentation for this
simulation and the UI itself is in Chinese. The way in which I wrote the code was tested on part of the ref system, but it seemed to
only be sending 2-3 particular messages and I could not test the full scope of the code. I feel I will be able to more
accurately test the code when I have a fully up and running ref system. 
*/

Ref_System::Ref_System(){
}

void Ref_System::init(S_RefSystem *tempInput){
	state = tempInput;
	Serial2.begin(115200);
}

bool Ref_System::read_serial(){

	byte enter_code, temp;          //These 3 lines initialize all our temporary variables
	uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;  
	uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;
	uint32_t temp_launch_speed;
   
	while(Serial2.available() > 1){
		enter_code = Serial2.read();
		
		if(enter_code == 0xA5){         //It looks for this value that signifies that there is about to be a transmission of data

			Serial2.readBytes(&temp, 1); 	// Get the length of the packet
			data_length = temp;
			Serial2.readBytes(&temp, 1);
			data_length = data_length | (temp << 8); 

			Serial2.readBytes(&temp, 1);
			seq = temp;
			Serial2.readBytes(&temp, 1);

			crc = temp;

			while(Serial2.readBytes(&temp, 1) != 1){}
			cmd_id = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			while(Serial2.readBytes(&temp, 1) != 1){}
			cmd_id = cmd_id | (temp << 8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			if(cmd_id == 0x202){   //power and heat data   
				 

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1){}
				state->chassis_voltage = temp_stat | (temp<<8);    

				////////////////////////////////////////////////////////////////////////////

				Serial2.readBytes(&temp, 1);
				temp_stat = temp;
				Serial2.readBytes(&temp, 1);    //Setting chasis output current
				state->chassis_current = temp_stat | (temp<<8);

				////////////////////////////////////////////////////////////////////////////
								
			}

			else if(cmd_id == 0x1){  //stage 1

				// Serial.println("received cmd_id inside 1"); 

				while(Serial2.readBytes(&temp, 1) != 1){}        //This waits till another byte of data is available

				comp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				comp_stat = comp_stat >> 4;

				if(comp_stat == 0)
					state->curr_stage = 'P';  //pre comp stage

				else if(comp_stat == 1)
					state->curr_stage = 'S';   //Setup

				else if(comp_stat == 2)
					state->curr_stage = 'I';    //Init stage

				else if(comp_stat == 3)
					state->curr_stage = 'F';   //5 sec countdown
				
				else if(comp_stat == 4)
					state->curr_stage = 'C';   //In combat

				else if(comp_stat == 5)
					state->curr_stage = 'R';   //calc comp results

				while(Serial2.readBytes(&temp, 1) != 1){}
				unix_time = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){}
				unix_time = unix_time | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->rem_time = int(unix_time);
									
			}

			else if(cmd_id == 0x2){   //results for 2

				//Serial.println("received cmd_id inside 2"); 
				while(Serial2.readBytes(&temp, 1) != 1){}
				comp_stat = temp;

				if(comp_stat == 0)
					state->comp_result = 'D';
					
				else if(comp_stat == 1)
					state->comp_result = 'R';
					
				else if(comp_stat == 2)
					state->comp_result = 'B';

			}

			// else if(cmd_id == 0x3){   //everyone hp

			// 	Serial.println("received cmd_id inside 3"); 

			// 	for (int i = 0; i < 16; i ++){
			// 		Serial2.readBytes(&temp, 1);
			// 		temp_hp = temp;
			// 		Serial2.readBytes(&temp, 1);
			// 		Serial.print("robot_id "); Serial.print(state->robot_id);
			// 		Serial.print(", "); Serial.println(i+1);
			// 		if (i + 1 == state->robot_id)
			// 		{	
			// 			Serial.println("robot health");
			// 			state->robot_health = temp_hp | (temp<<8);
			// 		}
			// 	}

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}		// Red hero HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// if (state->robot_id == 1)
			// 	// 	state->robot_health = temp_hp;

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}        //red infantry HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// if (state->robot_id == 3 or state->robot_id == 4 or state->robot_id == 5)
			// 	// 	state->robot_health = temp_hp;

			// 	// //////////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}        //red sentry HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// if (state->robot_id == 7)
			// 	// 	state->robot_health = temp_hp;

			// 	// //////////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}        //Blue Hero HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// state->blue_hero_hp = temp_hp;

			// 	// /////////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}

			// 	// ///////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}        //Blue Infantry HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// state->blue_infantry_hp = temp_hp;

			// 	// ///////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}

			// 	// //////////////////////////////////////////////////////////////////

			// 	// while(Serial2.readBytes(&temp, 1) != 1){}
			// 	// temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
			// 	// while(Serial2.readBytes(&temp, 1) != 1){}		//Blue Sentry HP
			// 	// temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// 	// state->blue_sentry_hp = temp_hp;
		   
			// 	///////////////////////////////////////////////////////////////
		
			// }

			// else if(cmd_id == 5){   //resto zone and everyone's projectivels

			// 	//Serial.println("received cmd_id inside 5"); 

			// ///////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){   
			// }
			// while(Serial2.readBytes(&temp, 1) != 1){   
			// }
			// while(Serial2.readBytes(&temp, 1) != 1){      //Skipping 3 bytes   
			// }
		
			// //////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){  
			// }        //This waits till another byte of data is available

			// rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){            //RED 1 remaining projectiles   
			// }        //This waits till another byte of data is available

			// rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			// state->red_one_rem_proj = temp_hp;

			// ///////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){   
			// }        //This waits till another byte of data is available

			// rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){            //RED 2 remaining projectiles    
			// }        //This waits till another byte of data is available

			// rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			// state->red_two_rem_proj = temp_hp;

			// ///////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){  
			// }        //This waits till another byte of data is available

			// rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){            //Blue one remaining projectiles
			// }        //This waits till another byte of data is available

			// rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			// state->blue_one_rem_proj = temp_hp;

			// ///////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){  
			// }        //This waits till another byte of data is available

			// rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){            //Blue two remaining projectiles  
			// }        //This waits till another byte of data is available

			// rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

			// state->blue_two_rem_proj = temp_hp;
		  
			// ///////////////////////////////////////////////////////////////
				
			// }

			// else if(cmd_id == 260){  //ref warning

			// //Serial.println("received cmd_id inside 260"); 


			// while(Serial2.readBytes(&temp, 1) != 1){  
			// }        //This waits till another byte of data is available

			// warning_level = temp;

			// if(warning_level == 1){

			// 	state->ref_warning = 'Y';
				
			// }else if(warning_level == 2){

			// 	state->ref_warning = 'R';
				
			// }else if(warning_level == 3){

			// 	state->ref_warning = 'F';

			// }

			// while(Serial2.readBytes(&temp, 1) != 1){  
			// }        //This waits till another byte of data is available

			// robo_id = temp;

			// state->foul_robot_id = int(robo_id);

			// }

			else if(cmd_id == 0x201){ //robo stat

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				state->robot_id = temp;
				while(Serial2.readBytes(&temp, 1) != 1){}
				state->robot_level = temp;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_hp = temp;
				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

				////////////////////////////////////////////////////////////////////////////
				
				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_max_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_max_hp = temp_max_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

				////////////////////////////////////////////////////////////////////////////

				// //This part looks for the robot id and then assigns values that follow depending on what robot id is transmitted

				if(int(robo_id) == 1){      //red hero

					if(int(state->robot_level) >= 1 && int(state->robot_level) <=3)
						state->robot_level = int(state->robot_level);

					state->red_hero_hp = temp_hp;
					state->red_hero_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 3){    //red infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						state->robot_level = int(robot_level);

					state->red_infantry_hp = temp_hp;
					state->red_infantry_max_hp = temp_max_hp;
				}
				else if(int(robo_id) == 7){    //red sentry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						state->robot_level = int(robot_level);

					state->red_sentry_hp = temp_hp;
					state->red_sentry_max_hp = temp_max_hp;
				}
				else if(int(robo_id) == 101){    //blue hero

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						state->robot_level = int(robot_level);      

					state->blue_hero_hp = temp_hp;
					state->blue_hero_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 103){    //blue infantry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						state->robot_level = int(robot_level);

					state->blue_infantry_hp = temp_hp;
					state->blue_infantry_max_hp = temp_max_hp;
					
				}
				else if(int(robo_id) == 107){    //blue sentry

					if(int(robot_level) >= 1 && int(robot_level) <=3)
						state->robot_level = int(robot_level);

					state->blue_sentry_hp = temp_hp;
					state->blue_sentry_max_hp = temp_max_hp;
					
				}

				////////////////////////////////////////////////////////////////////////////
			
				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} // robot _ 1 cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_1_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;
				while(Serial2.readBytes(&temp, 1) != 1){} // robot_1 barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_1_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} // setting robot 1 speed
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_1_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} // robot_2 cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_2_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} 	// robot 2 barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_2_barr_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} 	//robot 2 speed limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_2_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} 	// 42mm cooling value
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_42_cool_val = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){}	// 42mm barrel heat limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_42_heat_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){}		// robot 42mm speed limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_42_speed_lim = temp_stat;

				////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				while(Serial2.readBytes(&temp, 1) != 1){} // robot power limit
				temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer
				state->robot_power_lim = temp_stat;

				/////////////////////////////////////////////////////////////////////////////

				while(Serial2.readBytes(&temp, 1) != 1){}
				temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
				state->chassis_on = (temp_stat >> 0) & 1;
		  		state->gimbal_on = (temp_stat >> 1) & 1;
				state->shooter_on = (temp_stat >> 2) & 1;
			}

			else if (cmd_id == 0x204){
				Serial2.readBytes(&temp, 1);
				state->robot_health = temp;
			}

			//else if(cmd_id == 516){ //robot buffs

			// //Serial.println("received cmd_id inside 516"); 

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// if((temp & 0b00000001) == 1){
			// 	state->robot_buff = '0';
			// }else if((temp & 0b00000010) == 1){
			// 	state->robot_buff = '1';
			// }else if((temp & 0b00000100) == 1){
			// 	state->robot_buff = '2';
			// }else if((temp & 0b00001000) == 1){
			// 	state->robot_buff = '3';
			// }
			
			// }else if(cmd_id == 518){  //damage stats

			// //Serial.println("received cmd_id inside 518"); 

			// }else if(cmd_id == 519){  //RT launch info

			// //Serial.println("received cmd_id inside 519"); 

			// ///////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available        //Skipping 2 bytes of data

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// ///////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// comp_stat = int(temp);

			// state->launch_freq = comp_stat;

			// //////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// temp_launch_speed = temp;

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available        //This section reads in 4 bytes and assigns them to one uint32 variable

			// temp_launch_speed = temp_launch_speed | (temp<<8);
			// // temp_launch_speed << 8;

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// temp_launch_speed = temp_launch_speed | (temp<<8);
			// // temp_launch_speed << 8;

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// temp_launch_speed = temp_launch_speed | (temp<<8);
			// // temp_launch_speed << 8;

			// /////////////////////////////////////////////////////////////////

			// state->launch_speed = temp_launch_speed;
			
			// }else if(cmd_id == 520){  //remaining proj.

			// //Serial.println("received cmd_id inside 520"); 

			// ////////////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){ 
			// }        //This waits till another byte of data is available

			// temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){} 	//Setting robot power consumption limit 
			// //This waits till another byte of data is available

			// //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// state->rem_17_proj = temp_stat | (temp<<8);

			// /////////////////////////////////////////////////////////////////////////

			// while(Serial2.readBytes(&temp, 1) != 1){}
			// //This waits till another byte of data is available

			// temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

			// while(Serial2.readBytes(&temp, 1) != 1){}    //Setting robot power consumption limit 
			// //This waits till another byte of data is available

			// //Performing a bitwise or to join the 2 bytes into an 16 bit integer
			// state->rem_42_proj = temp_stat | (temp<<8);

			// /////////////////////////////////////////////////////////////////////////
			
			// }else if(cmd_id == 521){  //RFID stat

			// //Serial.println("received cmd_id inside 521");           //I am not sure if I need to record this

			//}   
		}
	}
	return 1;
}