#include <Arduino.h>

#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"

#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
#include "robot_comms/hid_report.h"
#include "robot_comms/hid_parser.h"
#include "motor_drivers/rm_can_interface.h"

#define CYCLE_TIME_US 1000
#define CYCLE_TIME_MS CYCLE_TIME_US / 1000
#define DEVICE_READ_RATE 1000

Device_Manager device_manager;

// void sensor_read() {
//   if (DURATION_US(device_timer, ARM_DWT_CYCCNT) >= DEVICE_READ_RATE) {
//     switch (device_switch) {
//       case 0:
//         imu.read_lsm6dsox_accel();
//         device_switch += 1;
//         break;

//       case 1:
//         imu.read_lsm6dsox_gyro();
//         device_switch += 1;
//         break;

//       case 2:
//         imu.read_lis3mdl();
//         device_switch += 1;
//         break;

//       case 3:
//         receiver.read();
//         device_switch = 0;
//         break;
//     }
//   }
// }

// Runs once
void setup() {

  // #ifdef USB_RAWHID

  Serial.begin(1000000);

  if (Serial)
    Serial.println("-- TEENSY SERIAL START --");

  // #endif

  setup_blink();  
}

// Runs continuously
void loop() {

  timer_set(1);

  // handle any hid input output
  // device_manager.read_sensor();
  // device_manager.push_can();
  // device_manager.check_hid();

  timer_wait_us(1, CYCLE_TIME_US)
}
