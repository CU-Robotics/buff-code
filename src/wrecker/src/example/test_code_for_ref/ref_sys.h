#ifndef ref_sys_h 
#define ref_sys_h

class Ref_sys{

    public:

        Ref_sys();      //non-parameterized constructor

        Ref_sys();      //parameterized constructor

        string get_comp_type();

        string get_curr_stage();

        int get_remaining_time();

        int get_sync_time_stamp();

        void set_comp_type(string input);

        void set_curr_stage(string input);

        void set_remaining_time(int input);

        void set_sync_time_stamp(int input);

        string get_comp_result();

        void set_comp_result(string input);

        int get_red_hero_hp();

        int get_red_sentry_hp();

        int get_red_infantry_hp();

        int get_blue_hero_hp();

        int get_blue_sentry_hp();

        int get_blue_infantry_hp();

        void set_red_hero_hp(int input);

        void set_red_sentry_hp(int input);

        void set_red_infantry_hp(int input);

        void set_blue_hero_hp(int input);

        void set_blue_sentry_hp(int input);

        void set_blue_infantry_hp(int input);

        string get_dart_launch_team();

        int get_remaining_time_when_launch();

        void set_dart_launch_team(string input);

        void set_remaining_time_when_launch(int input);

        int get_red_one_rem_proj();

        int get_red_two_rem_proj();

        int get_blue_one_rem_proj();

        int get_blue_two_rem_proj();

        void set_red_one_rem_proj(int input);

        void set_red_two_rem_proj(int input);

        void set_blue_one_rem_proj(int input);
        
        void set_blue_two_rem_proj(int input);

        string get_warning_level();

        int get_foul_rob_id();

        int get_fifteen_sec_count();

        void set_warning_level(string input);

        void set_foul_rob_id(int input);
        
        void set_fifteen_sec_count(int input);

        int get_curr_robot_id();

        int get_robot_level();

        int get_robot_remaining_hp();

        int get_robot_max_hp();

        void set_curr_robot_id(int input);

        void set_robot_level(int input);

        void set_robot_remaining_hp(int input);
        
        void set_robot_max_hp(int input);

        int get_robot_1_cool_val();       //17mm

        int get_robot_1_barr_heat_lim();       //17mm

        int get_robot_1_speed_lim();       //17mm

        int get_robot_2_cool_val();       //17mm

        int get_robot_2_barr_heat_lim();       //17mm

        int get_robot_2_speed_lim();       //17mm

        int get_robot_42_cool_val();

        int get_robot_42_heat_lim();

        int get_robot_42_speed_lim();

        void set_robot_1_cool_val(int input);       //17mm

        void set_robot_1_barr_heat_lim(int input);       //17mm

        void set_robot_1_speed_lim(int input);       //17mm

        void set_robot_2_cool_val(int input);       //17mm

        void set_robot_2_barr_heat_lim(int input);       //17mm

        void set_robot_2_speed_lim(int input);       //17mm

        void set_robot_42_cool_val(int input);

        void set_robot_42_heat_lim(int input);

        void set_robot_42_speed_lim(int input);

        int get_robot_power_lim();

        void set_robot_power_lim(int input);


    private:

        string comp_type;
        string curr_stage;
        int remaining_time;
        int sync_time_stamp;

        string comp_result;

        int red_hero_hp;
        int red_sentry_hp;
        int red_infantry_hp;
        int blue_hero_hp;
        int blue_sentry_hp;
        int blue_infantry_hp;

        string dart_launch_team;
        int remaining_time_when_launch;

        int red_one_rem_proj;
        int red_two_rem_proj;
        int blue_one_rem_proj;
        int blue_two_rem_proj;

        string warning_level;
        int foul_rob_id;

        int fifteen_sec_count;

        int curr_robot_id;
        int robot_level;
        int robot_remaining_hp;
        int robot_max_hp;

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



};


#endif