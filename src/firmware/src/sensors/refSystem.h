#ifndef REF_SYS_H
#define REF_SYS_H

struct RefData {
    char curr_stage = '\r';
    char comp_type = '\r';
    char comp_result = '\r';

    int rem_time = -1;

    int red_hero_hp = -1;
    int red_sentry_hp = -1;
    int red_infantry_hp = -1;
    int blue_hero_hp = -1;
    int blue_sentry_hp = -1;
    int blue_infantry_hp = -1;
    int red_hero_max_hp = -1;
    int red_sentry_max_hp = -1;
    int red_infantry_max_hp = -1;
    int blue_hero_max_hp = -1;
    int blue_sentry_max_hp = -1;
    int blue_infantry_max_hp = -1;

    int red_one_rem_proj = -1;
    int red_two_rem_proj = -1;
    int blue_one_rem_proj = -1;
    int blue_two_rem_proj = -1;

    char ref_warning = '\r';
    int foul_robot_id = -1;

    int robot_id = -1;
    int robot_level = -1;
    int robot_health = -1;

    int red_hero_robot_level = -1;
    int red_infantry_robot_level = -1;
    int red_sentry_robot_level = -1;
    int blue_hero_robot_level = -1;
    int blue_infantry_robot_level = -1;
    int blue_sentry_robot_level = -1;

    int robot_1_cool_val = -1;       //17mm
    int robot_1_barr_heat_lim = -1;       //17mm
    int robot_1_speed_lim = -1;       //17mm

    int robot_2_cool_val = -1;       //17mm
    int robot_2_barr_heat_lim = -1;       //17mm
    int robot_2_speed_lim = -1;       //17mm

    int robot_42_cool_val = -1;
    int robot_42_heat_lim = -1;
    int robot_42_speed_lim = -1;

    int robot_power_lim = -1;

    int chassis_voltage = -1;
    int chassis_current = -1;
    int power_buffer = -1;

    int robot_buff = -1;

    int launch_freq = -1;
    int launch_speed = -1;

    int rem_17_proj = -1;
    int rem_42_proj = -1;

    int chassis_on = -1;
    int gimbal_on = -1;
    int shooter_on = -1;
};

class RefSystem{
    public:
        RefSystem();
        void init();
        bool read_serial();
        void write_serial();

        byte* generate_hud_msg();
        byte* generate_movement_command_msg();
        byte* generate_state_msg();
        byte* generate_graphic(int, int, int, int, int, int, int, int, int, int, int, int, int);
        uint8_t retrieve_message_id();

        RefData data;
        uint8_t message_id = 0;

        // Display rendering
        bool mapMode = false;

        // Robot map
        float sentryPos[2];
        float sentryGoal[2];
        float sentrySelectedGoal[2];
        float sentryWaypoints[8][2];
        float infantryPos[2];
        float infantryGoal[2];
        float infantrySelectedGoal[2];
        float infantryWaypoints[8][2];
};

#endif