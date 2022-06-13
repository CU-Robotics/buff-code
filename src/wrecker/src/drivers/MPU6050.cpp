// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>        //Including the librariy for the Adafruit 6050 IMU
// #include <Wire.h>

#include "MPU6050.h"        //Including the header file for this driver

MPU6050::MPU6050(){         //Our default constructor
    current_index = 0;      //Sets the index of the circular array to head
}

void MPU6050::init(){       //Our init function 

    mpu.begin();        //Calling the mpu begin function that is included in the Adafruit libraries

    current_index = 0;    //this is for the circular array being set to the head

  //Serial.println("MPU6050 Found!");     //A serial print function for debugging

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  //Serial.println("");     //A serial print function for debugging
  delay(100);
}

bool MPU6050::update_MPU6050(){

    mpu.getEvent(&a_temp, &g_temp, &temp);           //calling the getEvent function which updates the sensor events passed through its parameters

    if(current_index < array_length){       //The case of we haven't reached the end of the array with the current index

        run_data_accel[current_index+1] = a_temp.acceleration;
        run_data_gyro[current_index+1] = g_temp.gyro;
        current_index++;

    }
    else if(current_index == array_length){     //The case where we have reached the end of the array with the current index

        run_data_accel[0] = a_temp.acceleration;
        run_data_gyro[0] = g_temp.gyro;
        current_index = 0;   //Setting it up so next time it runs from head + 1

    }

    a = run_data_accel[current_index];
    g = run_data_gyro[current_index];       //Putting the sensor event just filled into the circular array into a member variable that represents the current 
                                            //IMU status. This will be used later in the getters.

}

float MPU6050::get_accel_x(){
    return a.x;
}

float MPU6050::get_accel_y(){
    return a.y;
}

float MPU6050::get_accel_z(){               //The getters for the current IMU status
    return a.z;
}

float MPU6050::get_gyro_x(){
    return g.x;
}

float MPU6050::get_gyro_y(){
    return g.y;
}

float MPU6050::get_gyro_z(){
    return g.z;
}

float MPU6050::get_accel_previous_x(int how_many_back){         //The getters for the previous IMU states from the circular array

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        a_at = run_data_accel[current_index-how_many_back];      //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter

    }
    else{
        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            a_at = run_data_accel[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return a_at.x;
    
}

float MPU6050::get_accel_previous_y(int how_many_back){

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        a_at = run_data_accel[current_index-how_many_back];     //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter

    }
    else{
        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            a_at = run_data_accel[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return a_at.y;
}

float MPU6050::get_accel_previous_z(int how_many_back){

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        a_at = run_data_accel[current_index-how_many_back];       //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter

    }
    else{

        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            a_at = run_data_accel[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return a_at.z;
}

float MPU6050::get_gyro_previous_x(int how_many_back){

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        g_at = run_data_gyro[current_index-how_many_back];     //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter  

    }
    else{

        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            g_at = run_data_gyro[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return g_at.x;
}

float MPU6050::get_gyro_previous_y(int how_many_back){

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        g_at = run_data_gyro[current_index-how_many_back];      //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter

    }
    else{

        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            g_at = run_data_gyro[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return g_at.y;
}

float MPU6050::get_gyro_previous_z(int how_many_back){

    if((current_index - how_many_back)>=0){         //If you can go back from the current index the number passed through the parameter and still be greater than the head of the array

        g_at = run_data_gyro[current_index-how_many_back];     //Setting the variable to be returned equal to the current index minus the ammount passed throught the parameter  

    }
    else{

        if(array_length - (current_index - how_many_back)>current_index){       //If the array does have enough capacity to go that far back

            g_at = run_data_gyro[array_length - (current_index - how_many_back)];      //Setting the variable to be returned equal to the ammount wrapped back around the circular array.

        }else{

            return -1;      //The array does not have capacity to go that far back

        }
    }

    return g_at.z;
}