#include <Arduino.h>
#include <iostream>
#include "ref_sys.h"

/*
The methods in this code have been tested on a partial ref system, but this particular code has not been 
a full ref system. I attempted to test it on a full game mode system using the ref system simulation but, I could
not get the servers to work. There was also trouble getting this up and running since, all the documentation for this
simulation and the UI itself is in Chinese. The way in which I wrote the code was tested on part of the ref system, but it seemed to
only be sending 2-3 particular messages and I could not test the full scope of the code. I feel I will be able to more
accurately test the code when I have a fully up and running ref system. 
*/

ref_sys::ref_sys(){
    run_data -> curr_stage = 'Z';
    run_data -> comp_type= 'Z';
    run_data -> rem_time = 0;
    run_data -> comp_result = 'Z';

    run_data -> red_hero_hp = 0;
    run_data -> red_sentry_hp = 0;
    run_data -> red_infantry_hp = 0;
    run_data -> blue_hero_hp = 0;
    run_data -> blue_sentry_hp = 0;
    run_data -> blue_infantry_hp = 0;
    run_data -> red_hero_max_hp = 0;
    run_data -> red_sentry_max_hp = 0;
    run_data -> red_infantry_max_hp = 0;
    run_data -> blue_hero_max_hp = 0;
    run_data -> blue_sentry_max_hp = 0;
    run_data -> blue_infantry_max_hp = 0;

    run_data -> red_one_rem_proj = 0;
    run_data -> red_two_rem_proj = 0;
    run_data -> blue_one_rem_proj = 0;
    run_data -> blue_two_rem_proj = 0;

    run_data -> ref_warning = 'Z';
    run_data -> foul_robot_id = 0;
    
    run_data -> red_hero_robot_level = 0;
    run_data -> red_infantry_robot_level = 0;
    run_data -> red_sentry_robot_level = 0;
    run_data -> blue_hero_robot_level = 0;
    run_data -> blue_infantry_robot_level = 0;
    run_data -> blue_sentry_robot_level = 0;

    run_data -> robot_1_cool_val = 0;       //17mm
    run_data -> robot_1_barr_heat_lim = 0;       //17mm
    run_data -> robot_1_speed_lim = 0;       //17mm

    run_data -> robot_2_cool_val = 0;       //17mm
    run_data -> robot_2_barr_heat_lim = 0;       //17mm
    run_data -> robot_2_speed_lim = 0;       //17mm

    run_data -> robot_42_cool_val = 0;
    run_data -> robot_42_heat_lim = 0;
    run_data -> robot_42_speed_lim = 0;  

    run_data -> robot_power_lim = 0;

    run_data -> chasis_volt = 0;
    run_data -> chasis_current = 0;

    run_data -> robot_buff = 0;

    run_data -> launch_freq = 0;
    run_data -> launch_speed = 0;

    run_data -> rem_17_proj = 0;
    run_data -> rem_42_proj = 0;
}

void ref_sys::init(S_RefSystem *tempInput) {
    run_data = tempInput;
    Serial2.begin(115200);
}

bool ref_sys::read_serial(){

    byte enter_code, temp;          //These 3 lines initialize all our temporary variables
    uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;  
    uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;
    uint32_t temp_launch_speed;

    // ref_sys curr_ref;           //Setting up an instance of our ref_sys class
   
    // put your main code here, to run repeatedly:
    if(Serial2.available() > 1){      //The first instance to check if serial data is available

        bool gotStartByte = false;

        while(Serial2.available() > 1){
            enter_code = Serial2.read();    //It will read in the first byte of data
            
            if(enter_code == 0xA5){         //It looks for this value that signifies that there is about to be a transmission of data
                gotStartByte = true;

                //Serial.println("Enter code received");

                while(Serial2.readBytes(&temp, 1) != 1){        //This waits till another byte of data is available
                }
                
                data_length = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){        //This waits till another byte of data is available
                }
            
                data_length = data_length | (temp << 8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                Serial.println(data_length);

                while(Serial2.readBytes(&temp, 1) != 1){          //This waits till another byte of data is available
                }
                
                seq = temp;
                
                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                crc = temp;

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                cmd_id = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                cmd_id = cmd_id | (temp << 8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                Serial.println(cmd_id, HEX);


                if(cmd_id == 514){   //power and heat data   
                    
                //Serial.println("received cmd_id, inside 514"); 

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting chasis output voltage 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> chasis_volt = temp_stat;    

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting chasis output current
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> chasis_current = temp_stat;

                ////////////////////////////////////////////////////////////////////////////
                                    
                }

                else if(cmd_id == 1){  //stage 1

                //Serial.println("received cmd_id inside 1"); 

                while(Serial2.readBytes(&temp, 1) != 1){   
                }        //This waits till another byte of data is available

                comp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
                comp_stat = comp_stat >> 4;

                if(comp_stat == 0){
                    run_data -> curr_stage = 'P';  //pre comp stage
                    //Serial.println("pre comp");
                }

                else if(comp_stat == 1){
                    run_data -> curr_stage = 'S';   //Setup
                }

                else if(comp_stat == 2){
                    run_data -> curr_stage = 'I';    //Init stage
                }

                else if(comp_stat == 3){
                    run_data -> curr_stage = 'F';   //5 sec countdown
                }

                else if(comp_stat == 4){
                    run_data -> curr_stage = 'C';   //In combat
                }

                else if(comp_stat == 5){
                    run_data -> curr_stage = 'R';   //calc comp results
                }

                while(Serial2.readBytes(&temp, 1) != 1){   
                }

                unix_time = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    
                }        //This waits till another byte of data is available

                unix_time = unix_time | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> rem_time = int(unix_time);
                                    
                }else if(cmd_id == 2){   //results for 2

                //Serial.println("received cmd_id inside 2"); 

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                comp_stat = temp;

                if(comp_stat == 0){

                    run_data -> comp_result = 'D';
                    
                }else if(comp_stat == 1){

                    run_data -> comp_result = 'R';
                    
                }else if(comp_stat == 2){

                    run_data -> comp_result = 'B';
                    
                }
                    
                }else if(cmd_id == 3){   //everyone hp

                    //Serial.println("received cmd_id inside 3"); 

                    while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){   
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> red_hero_hp = temp_hp;

                /////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                while(Serial2.readBytes(&temp, 1) != 1){            //skipping 2 bytes  
                }        //This waits till another byte of data is available

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //red infantry hp  
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> red_infantry_hp = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }
                while(Serial2.readBytes(&temp, 1) != 1){  
                }
                while(Serial2.readBytes(&temp, 1) != 1){      //Skipping 4 bytes  
                }
                while(Serial2.readBytes(&temp, 1) != 1){ 
                }

                //////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //red infantry hp   
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> red_sentry_hp = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }
                while(Serial2.readBytes(&temp, 1) != 1){  
                }
                while(Serial2.readBytes(&temp, 1) != 1){      //Skipping 4 bytes   
                }
                while(Serial2.readBytes(&temp, 1) != 1){   
                }

                //////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Blue hero  
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> blue_hero_hp = temp_hp;

                /////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){   
                }
                while(Serial2.readBytes(&temp, 1) != 1){            //skipping 2 bytes   
                }

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //blue infantry hp
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> blue_infantry_hp = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }
                while(Serial2.readBytes(&temp, 1) != 1){   
                }
                while(Serial2.readBytes(&temp, 1) != 1){      //Skipping 4 bytes    
                }
                while(Serial2.readBytes(&temp, 1) != 1){   
                }

                //////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //blue infantry hp  
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> blue_sentry_hp = temp_hp;
               
                ///////////////////////////////////////////////////////////////
            
                }else if(cmd_id == 5){   //resto zone and everyone's projectivels

                    //Serial.println("received cmd_id inside 5"); 

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){   
                }
                while(Serial2.readBytes(&temp, 1) != 1){   
                }
                while(Serial2.readBytes(&temp, 1) != 1){      //Skipping 3 bytes   
                }
            
                //////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //RED 1 remaining projectiles   
                }        //This waits till another byte of data is available

                rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> red_one_rem_proj = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){   
                }        //This waits till another byte of data is available

                rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //RED 2 remaining projectiles    
                }        //This waits till another byte of data is available

                rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> red_two_rem_proj = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //Blue one remaining projectiles
                }        //This waits till another byte of data is available

                rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> blue_one_rem_proj = temp_hp;

                ///////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){            //Blue two remaining projectiles  
                }        //This waits till another byte of data is available

                rem_proj = rem_proj | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> blue_two_rem_proj = temp_hp;
              
                ///////////////////////////////////////////////////////////////
                    
                }else if(cmd_id == 260){ //ref warning

                //Serial.println("received cmd_id inside 260"); 


                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                warning_level = temp;

                if(warning_level == 1){

                    run_data -> ref_warning = 'Y';
                    
                }else if(warning_level == 2){

                    run_data -> ref_warning = 'R';
                    
                }else if(warning_level == 3){

                    run_data -> ref_warning = 'F';

                }

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                robo_id = temp;

                run_data -> foul_robot_id = int(robo_id);

                }else if(cmd_id == 201){ //robo stat

                //Serial.println("received cmd_id inside 201"); 

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                robo_id = temp;

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                robot_level = temp;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_hp = temp_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                ////////////////////////////////////////////////////////////////////////////
                
                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_max_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                temp_max_hp = temp_max_hp | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                ////////////////////////////////////////////////////////////////////////////

                //This part looks for the robot id and then assigns values that follow depending on what robot id is transmitted

                if(int(robo_id) == 1){      //red hero

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> red_hero_robot_level = int(robot_level);
                    }

                    run_data -> red_hero_hp = temp_hp;

                    run_data -> red_hero_max_hp = temp_max_hp;
                    
                }else if(int(robo_id) == 3){    //red infantry

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> red_infantry_robot_level = int(robot_level);
                    }

                    run_data -> red_infantry_hp = temp_hp;

                    run_data -> red_infantry_max_hp = temp_max_hp;
                    
                }else if(int(robo_id) == 7){    //red sentry

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> red_sentry_robot_level = int(robot_level);
                    }

                    run_data -> red_sentry_hp = temp_hp;

                    run_data -> red_sentry_max_hp = temp_max_hp;
                    
                }else if(int(robo_id) == 101){    //blue hero

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> blue_hero_robot_level = int(robot_level);      
                    }

                    run_data -> blue_hero_hp = temp_hp;

                    run_data -> blue_hero_max_hp = temp_max_hp;
                    
                }else if(int(robo_id) == 103){    //blue infantry

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> blue_infantry_robot_level = int(robot_level);
                    }

                    run_data -> blue_infantry_hp = temp_hp;

                    run_data -> blue_infantry_max_hp = temp_max_hp;
                    
                }else if(int(robo_id) == 107){    //blue sentry

                    if(int(robot_level) >= 1 && int(robot_level) <=3){
                    run_data -> blue_sentry_robot_level = int(robot_level);
                    }

                    run_data -> blue_sentry_hp = temp_hp;
                    
                    run_data -> blue_sentry_max_hp = temp_max_hp;
                    
                }

                ////////////////////////////////////////////////////////////////////////////
                
                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){          //Robot 1 cooling value read in 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_1_cool_val = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot one barrel heat limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_1_barr_heat_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot one speed limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_1_speed_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){  
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two cooling value  
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_2_cool_val = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two barrel heat limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_2_barr_heat_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two speed limit
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_2_speed_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm cooling value
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_42_cool_val = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm barrel heat limit  
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_42_heat_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm speed limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_42_speed_lim = temp_stat;

                ////////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> robot_power_lim = temp_stat;

                /////////////////////////////////////////////////////////////////////////////
              
                }else if(cmd_id == 516){ //robot buffs

                //Serial.println("received cmd_id inside 516"); 

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                if((temp & 0b00000001) == 1){
                    run_data -> robot_buff = '0';
                }else if((temp & 0b00000010) == 1){
                    run_data -> robot_buff = '1';
                }else if((temp & 0b00000100) == 1){
                    run_data -> robot_buff = '2';
                }else if((temp & 0b00001000) == 1){
                    run_data -> robot_buff = '3';
                }
                
                }else if(cmd_id == 518){  //damage stats

                //Serial.println("received cmd_id inside 518"); 

                }else if(cmd_id == 519){  //RT launch info

                //Serial.println("received cmd_id inside 519"); 

                ///////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available        //Skipping 2 bytes of data

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                ///////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                comp_stat = int(temp);

                run_data -> launch_freq = comp_stat;

                //////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_launch_speed = temp;

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available        //This section reads in 4 bytes and assigns them to one uint32 variable

                temp_launch_speed = temp_launch_speed | (temp<<8);
                // temp_launch_speed << 8;

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_launch_speed = temp_launch_speed | (temp<<8);
                // temp_launch_speed << 8;

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_launch_speed = temp_launch_speed | (temp<<8);
                // temp_launch_speed << 8;

                /////////////////////////////////////////////////////////////////

                run_data -> launch_speed = temp_launch_speed;
                
                }else if(cmd_id == 520){  //remaining proj.

                //Serial.println("received cmd_id inside 520"); 

                ////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> rem_17_proj = temp_stat;

                /////////////////////////////////////////////////////////////////////////

                while(Serial2.readBytes(&temp, 1) != 1){ 
                }        //This waits till another byte of data is available

                temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left

                while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
                }        //This waits till another byte of data is available

                temp_stat = temp_stat | (temp<<8);       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

                run_data -> rem_42_proj = temp_stat;

                /////////////////////////////////////////////////////////////////////////
                
                }else if(cmd_id == 521){  //RFID stat

                //Serial.println("received cmd_id inside 521");           //I am not sure if I need to record this

                
        
        }
        
        }

        }
        
  
  }
}
