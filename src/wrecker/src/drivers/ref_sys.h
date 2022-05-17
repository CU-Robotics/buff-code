#ifndef REF_SYS_H
#define REF_SYS_H

#include "state/state.h"

// struct ref_data{

//     char curr_stage;
//     char comp_type;
//     int rem_time;
//     char comp_result;
//     int red_hero_hp;
//     int red_sentry_hp;
//     int red_infantry_hp;
//     int blue_hero_hp;
//     int blue_sentry_hp;
//     int blue_infantry_hp;
//     int red_hero_max_hp;
//     int red_sentry_max_hp;
//     int red_infantry_max_hp;
//     int blue_hero_max_hp;
//     int blue_sentry_max_hp;
//     int blue_infantry_max_hp;
//     int red_one_rem_proj;
//     int red_two_rem_proj;
//     int blue_one_rem_proj;
//     int blue_two_rem_proj;
//     char ref_warning;
//     int foul_robot_id;
//     int red_hero_robot_level;
//     int red_infantry_robot_level;
//     int red_sentry_robot_level;
//     int blue_hero_robot_level;
//     int blue_infantry_robot_level;
//     int blue_sentry_robot_level;

//     int robot_1_cool_val;       //17mm
//     int robot_1_barr_heat_lim;       //17mm
//     int robot_1_speed_lim;       //17mm

//     int robot_2_cool_val;       //17mm
//     int robot_2_barr_heat_lim;       //17mm
//     int robot_2_speed_lim;       //17mm

//     int robot_42_cool_val;
//     int robot_42_heat_lim;
//     int robot_42_speed_lim;  

//     int robot_power_lim;

//     int chasis_volt;
//     int chasis_current;

//     int robot_buff;

//     int launch_freq;
//     int launch_speed;

//     int rem_17_proj;
//     int rem_42_proj;


// };

class ref_sys{

    public:

    ref_sys();

    void init(S_RefSystem *tempInput);

    bool read_serial();

    S_RefSystem * run_data;

    private:



};

#endif