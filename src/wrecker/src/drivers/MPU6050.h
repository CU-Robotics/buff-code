#ifndef MPU6050_H
#define MPU6050_H
#include <Adafruit_MPU6050.h>


//#include "state/state.h"

class MPU6050{

    public:

    MPU6050();

    void init();

    bool update_MPU6050();

    float get_accel_x();

    float get_accel_y();

    float get_accel_z();

    float get_gyro_x();

    float get_gyro_y();

    float get_gyro_z();

    float get_accel_previous_x(int how_many_back);

    float get_accel_previous_y(int how_many_back);

    float get_accel_previous_z(int how_many_back);

    float get_gyro_previous_x(int how_many_back);

    float get_gyro_previous_y(int how_many_back);

    float get_gyro_previous_z(int how_many_back);
    
    /*
    In order to change the number of IMU instances you would like to record in the array, you need to change the value of the array_length below
    and the sizes of both the run_data_accel array and the run_data_gyro array below.
    */

    int array_length = 50;

    int current_index;

    sensors_vec_t  run_data_accel[50];       // the length of these arrays has to be equal to the array_length member variable
    sensors_vec_t  run_data_gyro[50];

    private:
        Adafruit_MPU6050  mpu;
        sensors_vec_t a, g;
        sensors_vec_t a_at, g_at;

        sensors_event_t a_temp, g_temp, temp;       //Declaring sensor event variables to be passed into the get event function that is included in the adafruit library 


};

#endif