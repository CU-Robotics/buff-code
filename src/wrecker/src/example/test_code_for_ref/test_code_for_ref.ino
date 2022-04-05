byte enter_code;
uint16_t data_length, cmd_id, unix_time, temp_hp, rem_proj, temp_max_hp, temp_stat;
uint8_t seq, crc, comp_stat, warning_level, robo_id, robot_level;

class ref_system{

public:

void set_curr_stage(char temp){
  curr_stage = temp;
}

char get_curr_stage(){
  return curr_stage;
}

void set_rem_time(int input){
  rem_time = input;
}

int get_rem_time(){
  return rem_time;
}

void set_comp_result(char input){
  comp_result = input;
}

char get_comp_result(){
  return comp_result;
}

        int get_red_hero_hp(){

            return red_hero_hp;
        }

        int get_red_sentry_hp(){

            return red_sentry_hp;
        }

        int get_red_infantry_hp(){

            return red_infantry_hp;
        }

        int get_blue_hero_hp(){

            return blue_hero_hp;
        }

        int get_blue_sentry_hp(){

            return blue_sentry_hp;
        }

        int get_blue_infantry_hp(){

            return blue_infantry_hp;
        }

        void set_red_hero_hp(int input){

            red_hero_hp = input;
        }

        void set_red_sentry_hp(int input){

            red_sentry_hp = input;
        }

        void set_red_infantry_hp(int input){

            red_infantry_hp = input;
        }

        void set_blue_hero_hp(int input){

            blue_hero_hp = input;
        }

        void set_blue_sentry_hp(int input){

            blue_sentry_hp = input;
        }

        void set_blue_infantry_hp(int input){

            blue_infantry_hp = input;
        }

   int get_red_one_rem_proj(){

            return red_one_rem_proj;
        }

        int get_red_two_rem_proj(){

            return red_two_rem_proj;
        }

        int get_blue_one_rem_proj(){

            return blue_one_rem_proj;
        }

        int get_blue_two_rem_proj(){

            return blue_two_rem_proj;
        }

        void set_red_one_rem_proj(int input){
            
            red_one_rem_proj = input;
        }

        void set_red_two_rem_proj(int input){

            red_two_rem_proj = input;
        }

        void set_blue_one_rem_proj(int input){

            blue_one_rem_proj = input;
        }
        
        void set_blue_two_rem_proj(int input){

            blue_two_rem_proj = input;
        }  

   void set_ref_warning(char input){
      ref_warning = input;        
   }

   char get_ref_warning(){
    return ref_warning;
   }

   void set_foul_robot_id(int input){
    foul_robot_id = input;
   }

   int get_foul_robot_id(){
    return foul_robot_id;
   }

   void red_hero_set_robot_level(int input){
    red_hero_robot_level = input;
   }

   int red_hero_get_robot_level(){
    return red_hero_robot_level;
   }

   void red_infantry_set_robot_level(int input){
    red_infantry_robot_level = input;
   }

   int red_infantry_get_robot_level(){
    return red_infantry_robot_level;
   }

   void red_sentry_set_robot_level(int input){
    red_sentry_robot_level = input;
   }

   int red_sentry_get_robot_level(){
    return red_sentry_robot_level;
   }

   void blue_hero_set_robot_level(int input){
    blue_hero_robot_level = input;
   }

   int blue_hero_get_robot_level(){
    return blue_hero_robot_level;
   }

   void blue_infantry_set_robot_level(int input){
    blue_infantry_robot_level = input;
   }

   int blue_infantry_get_robot_level(){
    return blue_infantry_robot_level;
   }

   void blue_sentry_set_robot_level(int input){
    blue_sentry_robot_level = input;
   }

   int blue_sentry_get_robot_level(){
    return blue_sentry_robot_level;
   }

int get_red_hero_max_hp(){
  return red_hero_max_hp;
}
int get_red_sentry_max_hp(){
  return red_sentry_max_hp;
}
int get_red_infantry_max_hp(){
  return red_infantry_max_hp;
}
int get_blue_hero_max_hp(){
  return blue_hero_max_hp;
}
int get_blue_sentry_max_hp(){
  return blue_sentry_max_hp;
}
int get_blue_infantry_max_hp(){
  return blue_infantry_max_hp;
}
void set_red_hero_max_hp(int input){
  red_hero_max_hp = input;
}
void set_red_sentry_max_hp(int input){
  red_sentry_max_hp = input;
}
void set_red_infantry_max_hp(int input){
  red_infantry_max_hp = input;
}
void set_blue_hero_max_hp(int input){
  blue_hero_max_hp = input;
}
void set_blue_sentry_max_hp(int input){
  blue_sentry_max_hp = input;
}
void set_blue_infantry_max_hp(int input){
  blue_infantry_max_hp = input;
}

       int get_robot_1_cool_val(){
            return robot_1_cool_val;
        }       //17mm

        int get_robot_1_barr_heat_lim(){
            return robot_1_barr_heat_lim;
        }       //17mm

        int get_robot_1_speed_lim(){
            return robot_1_speed_lim;
        }       //17mm

        int get_robot_2_cool_val(){
            return robot_2_cool_val;
        }       //17mm

        int get_robot_2_barr_heat_lim(){
            return robot_2_barr_heat_lim;
        }       //17mm

        int get_robot_2_speed_lim(){
            return robot_2_speed_lim;
        }       //17mm

        int get_robot_42_cool_val(){
            return robot_42_cool_val;
        }

        int get_robot_42_barr_heat_lim(){
            return robot_42_heat_lim;
        }
        
        int get_robot_42_speed_lim(){
            return robot_42_speed_lim;
        }

        void set_robot_1_cool_val(int input){
            robot_1_cool_val = input;
        }       //17mm

        void set_robot_1_barr_heat_lim(int input){
            robot_1_barr_heat_lim = input;
        }       //17mm

        void set_robot_1_speed_lim(int input){
            robot_1_speed_lim = input;
        }       //17mm

        void set_robot_2_cool_val(int input){
            robot_2_cool_val = input;
        }       //17mm

        void set_robot_2_barr_heat_lim(int input){
            robot_2_barr_heat_lim = input;
        }       //17mm

        void set_robot_2_speed_lim(int input){
            robot_2_speed_lim = input;
        }       //17mm

        void set_robot_42_cool_val(int input){
            robot_42_cool_val = input;
        }

        void set_robot_42_barr_heat_lim(int input){
            robot_42_heat_lim = input;
        }

        void set_robot_42_speed_lim(int input){
            robot_42_speed_lim = input;
        }

  void set_robot_power_lim(int input){
         robot_power_lim = input;
   }

   int get_robot_power_lim(){
    return robot_power_lim;
   }

  int get_chasis_volt(){
     return chasis_volt;
  }
  int get_chasis_current(){
    return chasis_current;
  }
  void set_chasis_volt(int input){
     chasis_volt = input;
  }
  void set_chasis_current(int input){
    chasis_current = input;
  }

  int get_robot_buff(){
    return robot_buff;
  }

  void set_robot_buff(int temp){
    robot_buff = temp;
  }
   

private:

char curr_stage;
int rem_time;
char comp_result;
int red_hero_hp;
int red_sentry_hp;
int red_infantry_hp;
int blue_hero_hp;
int blue_sentry_hp;
int blue_infantry_hp;
int red_hero_max_hp;
int red_sentry_max_hp;
int red_infantry_max_hp;
int blue_hero_max_hp;
int blue_sentry_max_hp;
int blue_infantry_max_hp;
int red_one_rem_proj;
int red_two_rem_proj;
int blue_one_rem_proj;
int blue_two_rem_proj;
char ref_warning;
int foul_robot_id;
int red_hero_robot_level;
int red_infantry_robot_level;
int red_sentry_robot_level;
int blue_hero_robot_level;
int blue_infantry_robot_level;
int blue_sentry_robot_level;

int robot_1_cool_val;       //17mm
int robot_1_barr_heat_lim;       //17mm
int robot_1_speed_lim;       //17mm

int robot_2_cool_val;       //17mm
int robot_2_barr_heat_lim;       //17mm
int robot_2_speed_lim;       //17mm

int robot_42_cool_val;
int robot_42_heat_lim;
int robot_42_speed_lim;  

int robot_power_lim;

int chasis_volt;
int chasis_current;

int robot_buff;

};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

ref_system curr_ref;

void loop() {

  byte temp;
  
  
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
     else if(cmd_id == 1){  //stage

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

        
     }else if(cmd_id == 2){   //results

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
