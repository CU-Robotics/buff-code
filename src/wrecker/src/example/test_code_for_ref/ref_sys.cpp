#include <iostream>
#include <string>
#include "ref_sys.h"

using namespace std;


Ref_sys::Ref_sys(){

}      //non-parameterized constructor


string Ref_sys::get_comp_type(){

    return comp_type;
}

        string Ref_sys::get_curr_stage(){

            return curr_stage;
        }

        int Ref_sys::get_remaining_time(){

            return remaining_time;
        }

        int Ref_sys::get_sync_time_stamp(){

            return sync_time_stamp;
        }

        void Ref_sys::set_comp_type(string input){
            comp_type = input;
        }

        void Ref_sys::set_curr_stage(string input){
            curr_stage = input;
        }

        void Ref_sys::set_remaining_time(int input){
            remaining_time = input;
        }

        void Ref_sys::set_sync_time_stamp(int input){
            sync_time_stamp = input;
        }

        string Ref_sys::get_comp_result(){

            return comp_result;
        }

        void Ref_sys::set_comp_result(string input){

            comp_result = input;
        }

        int Ref_sys::get_red_hero_hp(){

            return red_hero_hp;
        }

        int Ref_sys::get_red_sentry_hp(){

            return red_sentry_hp;
        }

        int Ref_sys::get_red_infantry_hp(){

            return red_infantry_hp;
        }

        int Ref_sys::get_blue_hero_hp(){

            return blue_hero_hp;
        }

        int Ref_sys::get_blue_sentry_hp(){

            return blue_sentry_hp;
        }

        int Ref_sys::get_blue_infantry_hp(){

            return blue_infantry_hp;
        }

        void Ref_sys::set_red_hero_hp(int input){

            red_hero_hp = input;
        }

        void Ref_sys::set_red_sentry_hp(int input){

            red_sentry_hp = input;
        }

        void Ref_sys::set_red_infantry_hp(int input){

            red_infantry_hp = input;
        }

        void Ref_sys::set_blue_hero_hp(int input){

            blue_hero_hp = input;
        }

        void Ref_sys::set_blue_sentry_hp(int input){

            blue_sentry_hp = input;
        }

        void Ref_sys::set_blue_infantry_hp(int input){

            blue_infantry_hp = input;
        }

        string Ref_sys::get_dart_launch_team(){

            return dart_launch_team;
        }

        int Ref_sys::get_remaining_time_when_launch(){

            return remaining_time_when_launch;
        }

        void Ref_sys::set_dart_launch_team(string input){

            dart_launch_team = input;
        }

        void Ref_sys::set_remaining_time_when_launch(int input){

            remaining_time_when_launch = input;
        }

        int Ref_sys::get_red_one_rem_proj(){

            return red_one_rem_proj;
        }

        int Ref_sys::get_red_two_rem_proj(){

            return red_two_rem_proj;
        }

        int Ref_sys::get_blue_one_rem_proj(){

            return blue_one_rem_proj;
        }

        int Ref_sys::get_blue_two_rem_proj(){

            return blue_two_rem_proj;
        }

        void Ref_sys::set_red_one_rem_proj(int input){
            
            red_one_rem_proj = input;
        }

        void Ref_sys::set_red_two_rem_proj(int input){

            red_two_rem_proj = input;
        }

        void Ref_sys::set_blue_one_rem_proj(int input){

            blue_one_rem_proj = input;
        }
        
        void Ref_sys::set_blue_two_rem_proj(int input){

            blue_two_rem_proj = input;
        }

        string Ref_sys::get_warning_level(){

            return warning_level;
        }

        int Ref_sys::get_foul_rob_id(){

            return foul_rob_id;
        }

        int Ref_sys::get_fifteen_sec_count(){

            return fifteen_sec_count;
        }

        void Ref_sys::set_warning_level(string input){

            warning_level = input;
        }

        void Ref_sys::set_foul_rob_id(int input){

            foul_rob_id = input;
        }
        
        void Ref_sys::set_fifteen_sec_count(int input){

            fifteen_sec_count = input;
        }

        int Ref_sys::get_curr_robot_id(){

            return curr_robot_id;
        }

        int Ref_sys::get_robot_level(){

            return robot_level;
        }

        int Ref_sys::get_robot_remaining_hp(){

            return robot_remaining_hp;
        }

        int Ref_sys::get_robot_max_hp(){

            return robot_max_hp;
        }

        void Ref_sys::set_curr_robot_id(int input){

            curr_robot_id = input;
        }

        void Ref_sys::set_robot_level(int input){

            robot_level = input;
        }

        void Ref_sys::set_robot_remaining_hp(int input){

            robot_remaining_hp = input;
        }
        
        void Ref_sys::set_robot_max_hp(int input){

            robot_max_hp = input;
        }

        int Ref_sys::get_robot_1_cool_val(){
            return robot_1_cool_val;
        }       //17mm

        int Ref_sys::get_robot_1_barr_heat_lim(){
            return robot_1_barr_heat_lim;
        }       //17mm

        int Ref_sys::get_robot_1_speed_lim(){
            return robot_1_speed_lim;
        }       //17mm

        int Ref_sys::get_robot_2_cool_val(){
            return robot_2_cool_val;
        }       //17mm

        int Ref_sys::get_robot_2_barr_heat_lim(){
            return robot_2_barr_heat_lim;
        }       //17mm

        int Ref_sys::get_robot_2_speed_lim(){
            return robot_2_speed_lim;
        }       //17mm

        int Ref_sys::get_robot_42_cool_val(){
            return robot_42_cool_val;
        }

        int Ref_sys::get_robot_42_heat_lim(){
            return robot_42_heat_lim;
        }
        
        int Ref_sys::get_robot_42_speed_lim(){
            return robot_42_speed_lim;
        }

        void Ref_sys::set_robot_1_cool_val(int input){
            robot_1_cool_val = input;
        }       //17mm

        void Ref_sys::set_robot_1_barr_heat_lim(int input){
            robot_1_barr_heat_lim = input;
        }       //17mm

        void Ref_sys::set_robot_1_speed_lim(int input){
            robot_1_speed_lim = input;
        }       //17mm

        void Ref_sys::set_robot_2_cool_val(int input){
            robot_2_cool_val = input;
        }       //17mm

        void Ref_sys::set_robot_2_barr_heat_lim(int input){
            robot_2_barr_heat_lim = input;
        }       //17mm

        void Ref_sys::set_robot_2_speed_lim(int input){
            robot_2_speed_lim = input;
        }       //17mm

        void Ref_sys::set_robot_42_cool_val(int input){
            robot_42_cool_val = input;
        }

        void Ref_sys::set_robot_42_heat_lim(int input){
            robot_42_heat_lim = input;
        }

        void Ref_sys::set_robot_42_speed_lim(int input){
            robot_42_speed_lim = input;
        }

        int Ref_sys::get_robot_power_lim(){
            return robot_power_lim;
        }

        void Ref_sys::set_robot_power_lim(int input){
            robot_power_lim = input;
        }





