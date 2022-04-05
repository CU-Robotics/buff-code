#include <iostream>
#include "ref_sys.h"

ref_sys::ref_sys(){

    Serial.begin(115200);
    Serial1.begin(115200);

}

bool ref_sys::read_serial(){

    byte enter_code, temp;
    uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;
    uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;

    ref_sys curr_ref;           //Setting up an instance of our ref_sys class
  
  
    // put your main code here, to run repeatedly:
    if(Serial1.available()>1){

    enter_code = Serial1.read();

    if(enter_code == 0xA5){
      
      Serial.println("Enter code received");

        while(Serial1.readBytes(&temp, 1) != 1){
        }
        
        data_length = temp;
        data_length = data_length << 8;

        while(Serial1.readBytes(&temp, 1) != 1){
        }
      

        data_length = data_length | temp;

        Serial.println(data_length);

        while(Serial1.readBytes(&temp, 1) != 1){  
        }
        

        seq = temp;
        
        while(Serial1.readBytes(&temp, 1) != 1){
        }

        crc = temp;

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        cmd_id = temp;
        cmd_id = cmd_id << 8;

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        cmd_id = cmd_id | temp;

        Serial.println(cmd_id);


        if(cmd_id == 514){   //power and heat data   
            
        Serial.println("received cmd_id inside 514"); 

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting chasis output voltage 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_chasis_volt(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting chasis output current
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_chasis_current(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        Serial.flush();
            
        }

        else if(cmd_id == 1){  //stage 1

        Serial.println("received cmd_id inside 1"); 

        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        comp_stat = temp;
        comp_stat = comp_stat >> 4;

        if(comp_stat == 0){
            curr_ref.set_curr_stage('P');  //pre comp stage
            Serial.println("pre comp");
        }

        else if(comp_stat == 1){
            curr_ref.set_curr_stage('S');   //Setup
        }

        else if(comp_stat == 2){
            curr_ref.set_curr_stage('I');    //Init stage
        }

        else if(comp_stat == 3){
            curr_ref.set_curr_stage('F');   //5 sec countdown
        }

        else if(comp_stat == 4){
            curr_ref.set_curr_stage('C');   //In combat
        }

        else if(comp_stat == 5){
            curr_ref.set_curr_stage('R');   //calc comp results
        }

        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        unix_time = temp;
        unix_time = unix_time << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    
        }

        unix_time = unix_time | temp;

        curr_ref.set_rem_time(int(unix_time));

        Serial.flush();

            
        }else if(cmd_id == 2){   //results for 2

        Serial.println("received cmd_id inside 2"); 

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        comp_stat = temp;

        if(comp_stat == 0){

            curr_ref.set_comp_result('D');
            
        }else if(comp_stat == 1){

            curr_ref.set_comp_result('R');
            
        }else if(comp_stat == 2){

            curr_ref.set_comp_result('B');
            
        }
            
        }else if(cmd_id == 3){   //everyone hp

            Serial.println("received cmd_id inside 3"); 

            while(Serial1.readBytes(&temp, 1) != 1){
            
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_red_hero_hp(temp_hp);

        /////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        while(Serial1.readBytes(&temp, 1) != 1){            //skipping 2 bytes  
        }

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //red infantry hp  
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_red_infantry_hp(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }
        while(Serial1.readBytes(&temp, 1) != 1){  
        }
        while(Serial1.readBytes(&temp, 1) != 1){      //Skipping 4 bytes  
        }
        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        //////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //red infantry hp   
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_red_sentry_hp(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }
        while(Serial1.readBytes(&temp, 1) != 1){  
        }
        while(Serial1.readBytes(&temp, 1) != 1){      //Skipping 4 bytes   
        }
        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        //////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Blue hero  
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_blue_hero_hp(temp_hp);

        /////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){   
        }
        while(Serial1.readBytes(&temp, 1) != 1){            //skipping 2 bytes   
        }

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //blue infantry hp
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_blue_infantry_hp(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }
        while(Serial1.readBytes(&temp, 1) != 1){   
        }
        while(Serial1.readBytes(&temp, 1) != 1){      //Skipping 4 bytes    
        }
        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        //////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //blue infantry hp  
        }

        temp_hp = temp_hp | temp;

        curr_ref.set_blue_sentry_hp(temp_hp);

        Serial.flush();

        ///////////////////////////////////////////////////////////////
      
        }else if(cmd_id == 5){   //resto zone and everyone's projectivels

            Serial.println("received cmd_id inside 5"); 

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){   
        }
        while(Serial1.readBytes(&temp, 1) != 1){   
        }
        while(Serial1.readBytes(&temp, 1) != 1){      //Skipping 3 bytes   
        }
    
        //////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        rem_proj = temp;
        rem_proj = rem_proj << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //RED 1 remaining projectiles   
        }

        rem_proj = rem_proj | temp;

        curr_ref.set_red_one_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){   
        }

        rem_proj = temp;
        rem_proj = rem_proj << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //RED 2 remaining projectiles    
        }

        rem_proj = rem_proj | temp;

        curr_ref.set_red_two_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        rem_proj = temp;
        rem_proj = rem_proj << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //Blue one remaining projectiles
        }

        rem_proj = rem_proj | temp;

        curr_ref.set_blue_one_rem_proj(temp_hp);

        ///////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        rem_proj = temp;
        rem_proj = rem_proj << 8;

        while(Serial1.readBytes(&temp, 1) != 1){            //Blue two remaining projectiles  
        }

        rem_proj = rem_proj | temp;

        curr_ref.set_blue_two_rem_proj(temp_hp);

        Serial.flush();

        ///////////////////////////////////////////////////////////////
            
        }else if(cmd_id == 260){ //ref warning

        Serial.println("received cmd_id inside 260"); 


        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        warning_level = temp;

        if(warning_level == 1){

            curr_ref.set_ref_warning('Y');
            
        }else if(warning_level == 2){

            curr_ref.set_ref_warning('R');
            
        }else if(warning_level == 3){

            curr_ref.set_ref_warning('F');

        }

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        robo_id = temp;

        curr_ref.set_foul_robot_id(int(robo_id));

        Serial.flush();
        

        }else if(cmd_id == 201){ //robo stat

        Serial.println("received cmd_id inside 201"); 

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        robo_id = temp;

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        robot_level = temp;

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_hp = temp;
        temp_hp = temp_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_hp = temp_hp | temp;

        ////////////////////////////////////////////////////////////////////////////
        
        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_max_hp = temp;
        temp_max_hp = temp_max_hp << 8;

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        temp_max_hp = temp_max_hp | temp;

        ////////////////////////////////////////////////////////////////////////////

        if(int(robo_id) == 1){      //red hero

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.red_hero_set_robot_level(int(robot_level));
            }

            curr_ref.set_red_hero_hp(temp_hp);

            curr_ref.set_red_hero_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 3){    //red infantry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.red_infantry_set_robot_level(int(robot_level));
            }

            curr_ref.set_red_infantry_hp(temp_hp);

            curr_ref.set_red_infantry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 7){    //red sentry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.red_sentry_set_robot_level(int(robot_level));
            }

            curr_ref.set_red_sentry_hp(temp_hp);

            curr_ref.set_red_sentry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 101){    //blue hero

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.blue_hero_set_robot_level(int(robot_level));      
            }

            curr_ref.set_blue_hero_hp(temp_hp);

            curr_ref.set_blue_hero_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 103){    //blue infantry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.blue_infantry_set_robot_level(int(robot_level));
            }

            curr_ref.set_blue_infantry_hp(temp_hp);

            curr_ref.set_blue_infantry_max_hp(temp_max_hp);
            
        }else if(int(robo_id) == 107){    //blue sentry

            if(int(robot_level) >= 1 && int(robot_level) <=3){
            curr_ref.blue_sentry_set_robot_level(int(robot_level));
            }

            curr_ref.set_blue_sentry_hp(temp_hp);
            
            curr_ref.set_blue_sentry_max_hp(temp_max_hp);
            
        }

        ////////////////////////////////////////////////////////////////////////////
        
        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){          //Robot 1 cooling value read in 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_1_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot one barrel heat limit 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_1_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot one speed limit 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_1_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){  
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot two cooling value  
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_2_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot two barrel heat limit 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_2_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot two speed limit
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_2_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot 42mm cooling value
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_42_cool_val(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot 42mm barrel heat limit  
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_42_barr_heat_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot 42mm speed limit 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_42_speed_lim(temp_stat);

        ////////////////////////////////////////////////////////////////////////////

        while(Serial1.readBytes(&temp, 1) != 1){ 
        }

        temp_stat = temp;
        temp_stat = temp_stat << 8;

        while(Serial1.readBytes(&temp, 1) != 1){    //Setting robot power consumption limit 
        }

        temp_stat = temp_stat | temp;

        curr_ref.set_robot_power_lim(temp_stat);

        Serial.flush();

        ////////////////////////////////////////////////////////////////////////////

        

        /////////////////////////////////////////////////////////////////////////////

        
        }else if(cmd_id == 516){ //robot buffs

        Serial.println("received cmd_id inside 516"); 

        Serial.flush();

        
        }else if(cmd_id == 518){  //damage stats

        Serial.println("received cmd_id inside 518"); 

        Serial.flush();

        
        }else if(cmd_id == 519){  //RT launch info

        Serial.println("received cmd_id inside 519"); 

        Serial.flush();

        
        }else if(cmd_id == 520){  //remaining proj.

        Serial.println("received cmd_id inside 520"); 

        Serial.flush();

        
        }else if(cmd_id == 521){  //RFID stat

        Serial.println("received cmd_id inside 521"); 

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