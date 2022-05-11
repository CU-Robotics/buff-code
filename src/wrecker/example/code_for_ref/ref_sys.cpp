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

    Serial.begin(115200);
    Serial2.begin(115200);

}

bool ref_sys::read_serial(){

    byte enter_code, temp;          //These 3 lines initialize all our temporary variables
    uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;  
    uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;
    uint32_t temp_launch_speed;

    ref_sys curr_ref;           //Setting up an instance of our ref_sys class
  
  
    // put your main code here, to run repeatedly:
    if(Serial2.available()>1){      //The first instance to check if serial data is available

    enter_code = Serial2.read();    //It will read in the first byte of data

    if(enter_code == 0xA5){         //It looks for this value that signifies that there is about to be a transmission of data
      
      Serial.println("Enter code received");

        while(Serial2.readBytes(&temp, 1) != 1){        //This waits till another byte of data is available
        }
        
        data_length = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        data_length = data_length << 8;

        while(Serial2.readBytes(&temp, 1) != 1){        //This waits till another byte of data is available
        }
      

        data_length = data_length | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

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
        cmd_id = cmd_id << 8;

        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        cmd_id = cmd_id | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        Serial.println(cmd_id);


        if(cmd_id == 514){   //power and heat data   
            
        Serial.println("received cmd_id inside 514"); 

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting chasis output voltage 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_chasis_volt(temp_stat);    

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting chasis output current
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_chasis_current(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        Serial.flush();
            
        }

        else if(cmd_id == 1){  //stage 1

        Serial.println("received cmd_id inside 1"); 

        while(Serial2.readBytes(&temp, 1) != 1){   
        }        //This waits till another byte of data is available

        comp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        comp_stat = comp_stat >> 4;

        if(comp_stat == 0){
            set_curr_stage('P');  //pre comp stage
            Serial.println("pre comp");
        }

        else if(comp_stat == 1){
            set_curr_stage('S');   //Setup
        }

        else if(comp_stat == 2){
            set_curr_stage('I');    //Init stage
        }

        else if(comp_stat == 3){
            set_curr_stage('F');   //5 sec countdown
        }

        else if(comp_stat == 4){
            set_curr_stage('C');   //In combat
        }

        else if(comp_stat == 5){
            set_curr_stage('R');   //calc comp results
        }

        while(Serial2.readBytes(&temp, 1) != 1){   
        }

        unix_time = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        unix_time = unix_time << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    
        }        //This waits till another byte of data is available

        unix_time = unix_time | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_rem_time(int(unix_time));

        Serial.flush();

            
        }else if(cmd_id == 2){   //results for 2

        Serial.println("received cmd_id inside 2"); 

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        comp_stat = temp;

        if(comp_stat == 0){

            set_comp_result('D');
            
        }else if(comp_stat == 1){

            set_comp_result('R');
            
        }else if(comp_stat == 2){

            set_comp_result('B');
            
        }
            
        }else if(cmd_id == 3){   //everyone hp

            Serial.println("received cmd_id inside 3"); 

            while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){   
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_red_hero_hp(temp_hp);

        /////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        while(Serial2.readBytes(&temp, 1) != 1){            //skipping 2 bytes  
        }        //This waits till another byte of data is available

        ///////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //red infantry hp  
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_red_infantry_hp(temp_hp);

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
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //red infantry hp   
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_red_sentry_hp(temp_hp);

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
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Blue hero  
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_blue_hero_hp(temp_hp);

        /////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){   
        }
        while(Serial2.readBytes(&temp, 1) != 1){            //skipping 2 bytes   
        }

        ///////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //blue infantry hp
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_blue_infantry_hp(temp_hp);

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
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //blue infantry hp  
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_blue_sentry_hp(temp_hp);

        Serial.flush();

        ///////////////////////////////////////////////////////////////
      
        }else if(cmd_id == 5){   //resto zone and everyone's projectivels

            Serial.println("received cmd_id inside 5"); 

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
        rem_proj = rem_proj << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //RED 1 remaining projectiles   
        }        //This waits till another byte of data is available

        rem_proj = rem_proj | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_red_one_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){   
        }        //This waits till another byte of data is available

        rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        rem_proj = rem_proj << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //RED 2 remaining projectiles    
        }        //This waits till another byte of data is available

        rem_proj = rem_proj | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_red_two_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        rem_proj = rem_proj << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //Blue one remaining projectiles
        }        //This waits till another byte of data is available

        rem_proj = rem_proj | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_blue_one_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        rem_proj = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        rem_proj = rem_proj << 8;

        while(Serial2.readBytes(&temp, 1) != 1){            //Blue two remaining projectiles  
        }        //This waits till another byte of data is available

        rem_proj = rem_proj | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_blue_two_rem_proj(temp_hp);

        Serial.flush();

        ///////////////////////////////////////////////////////////////
            
        }else if(cmd_id == 260){ //ref warning

        Serial.println("received cmd_id inside 260"); 


        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        warning_level = temp;

        if(warning_level == 1){

            set_ref_warning('Y');
            
        }else if(warning_level == 2){

            set_ref_warning('R');
            
        }else if(warning_level == 3){

            set_ref_warning('F');

        }

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        robo_id = temp;

        set_foul_robot_id(int(robo_id));

        Serial.flush();
        

        }else if(cmd_id == 201){ //robo stat

        Serial.println("received cmd_id inside 201"); 

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
        temp_hp = temp_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_hp = temp_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        ////////////////////////////////////////////////////////////////////////////
        
        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_max_hp = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_max_hp = temp_max_hp << 8;

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        temp_max_hp = temp_max_hp | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        ////////////////////////////////////////////////////////////////////////////

        //This part looks for the robot id and then assigns values that follow depending on what robot id is transmitted

        if(int(robo_id) == 1){      //red hero

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            red_hero_set_robot_level(int(robot_level));
            }

            set_red_hero_hp(temp_hp);

            set_red_hero_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 3){    //red infantry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            red_infantry_set_robot_level(int(robot_level));
            }

            set_red_infantry_hp(temp_hp);

            set_red_infantry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 7){    //red sentry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            red_sentry_set_robot_level(int(robot_level));
            }

            set_red_sentry_hp(temp_hp);

            set_red_sentry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 101){    //blue hero

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            blue_hero_set_robot_level(int(robot_level));      
            }

            set_blue_hero_hp(temp_hp);

            set_blue_hero_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 103){    //blue infantry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            blue_infantry_set_robot_level(int(robot_level));
            }

            set_blue_infantry_hp(temp_hp);

            set_blue_infantry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 107){    //blue sentry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            blue_sentry_set_robot_level(int(robot_level));
            }

            set_blue_sentry_hp(temp_hp);
            
            set_blue_sentry_max_hp(temp_max_hp);
            
        }

        ////////////////////////////////////////////////////////////////////////////
        
        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){          //Robot 1 cooling value read in 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_1_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot one barrel heat limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_1_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot one speed limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_1_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){  
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two cooling value  
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_2_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two barrel heat limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_2_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot two speed limit
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_2_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm cooling value
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_42_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm barrel heat limit  
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_42_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot 42mm speed limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_42_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_robot_power_lim(temp_stat);

        Serial.flush();

        ////////////////////////////////////////////////////////////////////////////

        

        /////////////////////////////////////////////////////////////////////////////

        
        }else if(cmd_id == 516){ //robot buffs

        Serial.println("received cmd_id inside 516"); 

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available


        if(temp[0] == 1){
            set_robot_buff('0');
        }else if(temp[1] == 1){
            set_robot_buff('1');
        }else if(temp[2] == 1){
            set_robot_buff('2');
        }else if(temp[3] == 1){
            set_robot_buff('3');
        }

        Serial.flush();

        
        }else if(cmd_id == 518){  //damage stats

        Serial.println("received cmd_id inside 518"); 


        Serial.flush();

        
        }else if(cmd_id == 519){  //RT launch info

        Serial.println("received cmd_id inside 519"); 

        ///////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available        //Skipping 2 bytes of data

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        ///////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available


        comp_stat = int(temp);

        set_launch_freq(comp_stat);

        //////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_launch_speed = temp;
        temp_launch_speed << 8;

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available        //This section reads in 4 bytes and assigns them to one uint32 variable

        temp_launch_speed = temp_launch_speed | temp;
        temp_launch_speed << 8;

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_launch_speed = temp_launch_speed | temp;
        temp_launch_speed << 8;

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_launch_speed = temp_launch_speed | temp;
        temp_launch_speed << 8;


        /////////////////////////////////////////////////////////////////

        set_launch_speed(temp_launch_speed);

        Serial.flush();

        
        }else if(cmd_id == 520){  //remaining proj.

        Serial.println("received cmd_id inside 520"); 

        ////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_rem_17_proj(temp_stat);

        /////////////////////////////////////////////////////////////////////////

        while(Serial2.readBytes(&temp, 1) != 1){ 
        }        //This waits till another byte of data is available

        temp_stat = temp;     //Reading in a byte of data and bit shifting it 8 bits to the left
        temp_stat = temp_stat << 8;

        while(Serial2.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
        }        //This waits till another byte of data is available

        temp_stat = temp_stat | temp;       //Performing a bitwise or to join the 2 bytes into an 16 bit integer

        set_rem_42_proj(temp_stat);

        /////////////////////////////////////////////////////////////////////////

        Serial.flush();

        
        }else if(cmd_id == 521){  //RFID stat

        Serial.println("received cmd_id inside 521");           //I am not sure if I need to record this

        Serial.flush();

      
     }
      
    }
  }
}

void ref_sys::set_curr_stage(char temp){
  run_data->curr_stage = temp;
}

char ref_sys::get_curr_stage(){
  return run_data->curr_stage;
}

void ref_sys::set_comp_type(char temp){
    run_data->comp_type = temp;
}

void ref_sys::set_rem_time(int input){
  run_data->rem_time = input;
}

int ref_sys::get_rem_time(){
  return run_data->rem_time;
}

void ref_sys::set_comp_result(char input){
  run_data->comp_result = input;
}

char ref_sys::get_comp_result(){
  return run_data->comp_result;
}

int ref_sys::get_red_hero_hp(){

    return run_data->red_hero_hp;
}

int ref_sys::get_red_sentry_hp(){

    return run_data->red_sentry_hp;
}

int ref_sys::get_red_infantry_hp(){

    return run_data->red_infantry_hp;
}

int ref_sys::get_blue_hero_hp(){

    return run_data->blue_hero_hp;
}

int ref_sys::get_blue_sentry_hp(){

    return run_data->blue_sentry_hp;
}

int ref_sys::get_blue_infantry_hp(){

    return run_data->blue_infantry_hp;
}

void ref_sys::set_red_hero_hp(int input){

    run_data->red_hero_hp = input;
}

void ref_sys::set_red_sentry_hp(int input){

    run_data->red_sentry_hp = input;
}

void ref_sys::set_red_infantry_hp(int input){

    run_data->red_infantry_hp = input;
}

void ref_sys::set_blue_hero_hp(int input){

    run_data->blue_hero_hp = input;
}

void ref_sys::set_blue_sentry_hp(int input){

    run_data->blue_sentry_hp = input;
}

void ref_sys::set_blue_infantry_hp(int input){

    run_data->blue_infantry_hp = input;
}

int ref_sys::get_red_one_rem_proj(){

    return run_data->red_one_rem_proj;
}

int ref_sys::get_red_two_rem_proj(){

    return run_data->red_two_rem_proj;
}

int ref_sys::get_blue_one_rem_proj(){

    return run_data->blue_one_rem_proj;
}

int ref_sys::get_blue_two_rem_proj(){

    return run_data->blue_two_rem_proj;
}

void ref_sys::set_red_one_rem_proj(int input){
    
    run_data->red_one_rem_proj = input;
}

void ref_sys::set_red_two_rem_proj(int input){

    run_data->red_two_rem_proj = input;
}

void ref_sys::set_blue_one_rem_proj(int input){

    run_data->blue_one_rem_proj = input;
}

void ref_sys::set_blue_two_rem_proj(int input){

    run_data->blue_two_rem_proj = input;
}  

void ref_sys::set_ref_warning(char input){
    run_data->ref_warning = input;        
}

char ref_sys::get_ref_warning(){
return run_data->ref_warning;
}

void ref_sys::set_foul_robot_id(int input){
run_data->foul_robot_id = input;
}

int ref_sys::get_foul_robot_id(){
return run_data->foul_robot_id;
}

void ref_sys::red_hero_set_robot_level(int input){
run_data->red_hero_robot_level = input;
}

int ref_sys::red_hero_get_robot_level(){
return run_data->red_hero_robot_level;
}

void ref_sys::red_infantry_set_robot_level(int input){
run_data->red_infantry_robot_level = input;
}

int ref_sys::red_infantry_get_robot_level(){
return run_data->red_infantry_robot_level;
}

void ref_sys::red_sentry_set_robot_level(int input){
run_data->red_sentry_robot_level = input;
}

int ref_sys::red_sentry_get_robot_level(){
return run_data->red_sentry_robot_level;
}

void ref_sys::blue_hero_set_robot_level(int input){
run_data->blue_hero_robot_level = input;
}

int ref_sys::blue_hero_get_robot_level(){
return run_data->blue_hero_robot_level;
}

void ref_sys::blue_infantry_set_robot_level(int input){
run_data->blue_infantry_robot_level = input;
}

int ref_sys::blue_infantry_get_robot_level(){
return run_data->blue_infantry_robot_level;
}

void ref_sys::blue_sentry_set_robot_level(int input){
run_data->blue_sentry_robot_level = input;
}

int ref_sys::blue_sentry_get_robot_level(){
return run_data->blue_sentry_robot_level;
}

int ref_sys::get_red_hero_max_hp(){
  return run_data->red_hero_max_hp;
}

int ref_sys::get_red_sentry_max_hp(){
  return run_data->red_sentry_max_hp;
}

int ref_sys::get_red_infantry_max_hp(){
  return run_data->red_infantry_max_hp;
}

int ref_sys::get_blue_hero_max_hp(){
  return run_data->blue_hero_max_hp;
}

int ref_sys::get_blue_sentry_max_hp(){
  return run_data->blue_sentry_max_hp;
}

int ref_sys::get_blue_infantry_max_hp(){
  return run_data->blue_infantry_max_hp;
}

void ref_sys::set_red_hero_max_hp(int input){
  run_data->red_hero_max_hp = input;
}

void ref_sys::set_red_sentry_max_hp(int input){
  run_data->red_sentry_max_hp = input;
}

void ref_sys::set_red_infantry_max_hp(int input){
  run_data->red_infantry_max_hp = input;
}

void ref_sys::set_blue_hero_max_hp(int input){
  run_data->blue_hero_max_hp = input;
}

void ref_sys::set_blue_sentry_max_hp(int input){
  run_data->blue_sentry_max_hp = input;
}

void ref_sys::set_blue_infantry_max_hp(int input){
  run_data->blue_infantry_max_hp = input;
}

int ref_sys::get_robot_1_cool_val(){
    return run_data->robot_1_cool_val;
}       //17mm

int ref_sys::get_robot_1_barr_heat_lim(){
    return run_data->robot_1_barr_heat_lim;
}       //17mm

int ref_sys::get_robot_1_speed_lim(){
    return run_data->robot_1_speed_lim;
}       //17mm

int ref_sys::get_robot_2_cool_val(){
    return run_data->robot_2_cool_val;
}       //17mm

int ref_sys::get_robot_2_barr_heat_lim(){
    return run_data->robot_2_barr_heat_lim;
}       //17mm

int ref_sys::get_robot_2_speed_lim(){
    return run_data->robot_2_speed_lim;
}       //17mm

int ref_sys::get_robot_42_cool_val(){
    return run_data->robot_42_cool_val;
}

int ref_sys::get_robot_42_barr_heat_lim(){
    return run_data->robot_42_heat_lim;
}

int ref_sys::get_robot_42_speed_lim(){
    return run_data->robot_42_speed_lim;
}

void ref_sys::set_robot_1_cool_val(int input){
    run_data->robot_1_cool_val = input;
}       //17mm

void ref_sys::set_robot_1_barr_heat_lim(int input){
    run_data->robot_1_barr_heat_lim = input;
}       //17mm

void ref_sys::set_robot_1_speed_lim(int input){
    run_data->robot_1_speed_lim = input;
}       //17mm

void ref_sys::set_robot_2_cool_val(int input){
    run_data->robot_2_cool_val = input;
}       //17mm

void ref_sys::set_robot_2_barr_heat_lim(int input){
    run_data->robot_2_barr_heat_lim = input;
}       //17mm

void ref_sys::set_robot_2_speed_lim(int input){
    run_data->robot_2_speed_lim = input;
}       //17mm

void ref_sys::set_robot_42_cool_val(int input){
    run_data->robot_42_cool_val = input;
}

void ref_sys::set_robot_42_barr_heat_lim(int input){
    run_data->robot_42_heat_lim = input;
}

void ref_sys::set_robot_42_speed_lim(int input){
    run_data->robot_42_speed_lim = input;
}

void ref_sys::set_robot_power_lim(int input){
        run_data->robot_power_lim = input;
}

int ref_sys::get_robot_power_lim(){
return run_data->robot_power_lim;
}

int ref_sys::get_chasis_volt(){
    return run_data->chasis_volt;
}

int ref_sys::get_chasis_current(){
return run_data->chasis_current;
}

void ref_sys::set_chasis_volt(int input){
    run_data->chasis_volt = input;
}

void ref_sys::set_chasis_current(int input){
run_data->chasis_current = input;
}

int ref_sys::get_robot_buff(){
return run_data->robot_buff;
}

void ref_sys::set_robot_buff(int temp){
run_data->robot_buff = temp;
}

int ref_sys::get_launch_freq(){
    return run_data->launch_freq;
}

int ref_sys::get_launch_speed(){
    return run_data -> launch_speed;
}

void ref_sys::set_launch_freq(int temp){
    run_data->launch_freq = temp;
}

void ref_sys::set_launch_speed(int temp){
    run_data -> launch_speed = temp;
}

int ref_sys::get_rem_17_proj(){
    return run_data -> rem_17_proj;
}

int ref_sys::get_rem_42_proj(){
    return run_data -> rem_42_proj;
}

void ref_sys::set_rem_17_proj(int temp){
    run_data -> rem_17_proj = temp;
}
    
void ref_sys::set_rem_42_proj(int temp){
    run_data -> rem_42_proj = temp;
}