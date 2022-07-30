
struct ref_data{

    char curr_stage;
    char comp_type;
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

    int launch_freq;
    int launch_speed;

    int rem_17_proj;
    int rem_42_proj;


};

class ref_sys{

    public:

    ref_sys();

    bool read_serial();

    void set_curr_stage(char temp);

    char get_curr_stage();

    void set_comp_type(char temp);

    void set_rem_time(int input);

    int get_rem_time();

    void set_comp_result(char input);

    char get_comp_result();

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

    int get_red_one_rem_proj();

    int get_red_two_rem_proj();

    int get_blue_one_rem_proj();

    int get_blue_two_rem_proj();

    void set_red_one_rem_proj(int input);

    void set_red_two_rem_proj(int input);

    void set_blue_one_rem_proj(int input);
        
    void set_blue_two_rem_proj(int input); 

    void set_ref_warning(char input);

    char get_ref_warning();

    void set_foul_robot_id(int input);

    int get_foul_robot_id();

    void red_hero_set_robot_level(int input);

    int red_hero_get_robot_level();

    void red_infantry_set_robot_level(int input);

    int red_infantry_get_robot_level();

    void red_sentry_set_robot_level(int input);

    int red_sentry_get_robot_level();

    void blue_hero_set_robot_level(int input);

    int blue_hero_get_robot_level();

    void blue_infantry_set_robot_level(int input);

    int blue_infantry_get_robot_level();

    void blue_sentry_set_robot_level(int input);

    int blue_sentry_get_robot_level();

    int get_red_hero_max_hp();

    int get_red_sentry_max_hp();

    int get_red_infantry_max_hp();

    int get_blue_hero_max_hp();

    int get_blue_sentry_max_hp();

    int get_blue_infantry_max_hp();

    void set_red_hero_max_hp(int input);

    void set_red_sentry_max_hp(int input);

    void set_red_infantry_max_hp(int input);

    void set_blue_hero_max_hp(int input);

    void set_blue_sentry_max_hp(int input);

    void set_blue_infantry_max_hp(int input);

    int get_robot_1_cool_val();       //17mm

    int get_robot_1_barr_heat_lim();       //17mm

    int get_robot_1_speed_lim();      //17mm

    int get_robot_2_cool_val();       //17mm

    int get_robot_2_barr_heat_lim();      //17mm

    int get_robot_2_speed_lim();       //17mm

    int get_robot_42_cool_val();

    int get_robot_42_barr_heat_lim();
        
    int get_robot_42_speed_lim();

    void set_robot_1_cool_val(int input);       //17mm

    void set_robot_1_barr_heat_lim(int input);       //17mm

    void set_robot_1_speed_lim(int input);       //17mm

    void set_robot_2_cool_val(int input);       //17mm

    void set_robot_2_barr_heat_lim(int input);       //17mm

    void set_robot_2_speed_lim(int input);       //17mm

    void set_robot_42_cool_val(int input);

    void set_robot_42_barr_heat_lim(int input);

    void set_robot_42_speed_lim(int input);

    void set_robot_power_lim(int input);

    int get_robot_power_lim();

    int get_chasis_volt();

    int get_chasis_current();

    void set_chasis_volt(int input);

    void set_chasis_current(int input);

    int get_robot_buff();

    void set_robot_buff(int temp);

    int get_launch_freq();

    int get_launch_speed();

    void set_launch_freq(int temp);

    void set_launch_speed(int temp);

    int get_rem_17_proj();

    int get_rem_42_proj();

    void set_rem_17_proj(int temp);
    
    void set_rem_42_proj(int temp);

    private:

    ref_data * run_data;


};