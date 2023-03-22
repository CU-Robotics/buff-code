//#include "global_robot_state.h"

#include "algorithms/pid_filter.h"
#include "algorithms/rolling_average_filter.h"
#include "motor_drivers/rm_can_interface.h"
#include "sensors/dr16.h"
#include "sensors/lsm6dsox.h"
#include "sensors/revEnc.h"

#include "buff_cpp/global_robot_state.h"

GlobalRobotState state;

uint32_t loopFrequency = 2000; // in microseconds
uint32_t programTime; // stores the system time at the start of every loop

float energyBuffer = 0;
RollingAverageFilter IMUFilter(10);

void setup() {
  delay(500); // Half-second startup delay

  Serial.begin(1000000);
	Serial.println("-- STANDARD ROBOT TEST PROGRAM --\n");

  // Serial.print("Waiting for sensor initialization");
  // while ( {Sensors are outputting garbage data} ) {
  //   Serial.print(".");
  //   delay(100);
  // }
  // Serial.print("Complete!\n");

  programTime = micros();
}

void demoLoop() {
  /* Chassis */
  float x = state.receiver.out.r_stick_x;
  float y = state.receiver.out.r_stick_y;
  float spin = -state.receiver.out.wheel;

  double speed_fr = y - x - spin;
  double speed_fl = -(y + x + spin);
  double speed_bl = -(y - x + spin);
  double speed_br = y + x - spin;

  if (isnan(speed_fr)) speed_fr = 0;
  if (isnan(speed_fl)) speed_fl = 0;
  if (isnan(speed_bl)) speed_bl = 0;
  if (isnan(speed_br)) speed_br = 0;

  double max_speed = fabs(speed_fr);
  max_speed = max(max_speed, fabs(speed_fl));
  max_speed = max(max_speed, fabs(speed_bl));
  max_speed = max(max_speed, fabs(speed_br));

  if (max_speed > 1.0) {
    speed_fr /= max_speed;
    speed_fl /= max_speed;
    speed_bl /= max_speed;
    speed_br /= max_speed;
  }

  float energyBufferCritThreshold = 30.0;
  float energyBufferLimitThreshold = 60.0;
  energyBuffer = state.ref.data.power_buffer;
  float ratio = 1.0;
  if (energyBuffer < energyBufferLimitThreshold) {
    ratio = constrain((energyBuffer - energyBufferCritThreshold) / energyBufferLimitThreshold, 0.0, 1.0);
  }

  float powerDraw = state.ref.data.chassis_current * state.ref.data.chassis_voltage / 1000000.0;
  float powerLimit = state.ref.data.robot_power_lim;
  float raw_fr = state.generateMotorRPMOutput(1, speed_fr * DEMO_CHASSIS_MAX_RPM) * ratio;
  float raw_fl = state.generateMotorRPMOutput(2, speed_fl * DEMO_CHASSIS_MAX_RPM) * ratio;
  float raw_bl = state.generateMotorRPMOutput(3, speed_bl * DEMO_CHASSIS_MAX_RPM) * ratio;
  float raw_br = state.generateMotorRPMOutput(7, speed_br * DEMO_CHASSIS_MAX_RPM) * ratio;

  state.rmCAN.set_output(1, raw_fr);
  state.rmCAN.set_output(2, raw_fl);
  state.rmCAN.set_output(3, raw_bl);
  state.rmCAN.set_output(7, raw_br);

  // /* Gimbal */
  if (state.receiver.out.l_stick_y != -1.0) {
    // Manual control
    float ratio = 17.0 / 246.0;
    IMUFilter.addMeasurement(state.imu.data[5]);
    float rpm = IMUFilter.getMean() / 6.0 / ratio;

    rpm += state.receiver.out.l_stick_x * DEMO_GIMBAL_YAW_MAX_RPM;

    state.setMotorRPM(5, rpm);
    state.setMotorRPM(6, rpm);
  } else {
    state.setMotorRPM(5, 0);
    state.setMotorRPM(6, 0);
  }

  // /* Shooter */
  if (state.receiver.out.r_switch == 1) {
    state.setMotorRPM(11, 8500.0);
    state.setMotorRPM(12, -8500.0);
    state.setMotorRPM(13, -150.0 * 36);
    state.setMotorRPM(14, 150.0 * 36);
  } else if (state.receiver.out.r_switch == 3) {
    state.setMotorRPM(11, 8500.0);
    state.setMotorRPM(12, -8500.0);
    state.setMotorRPM(13, 0);
    state.setMotorRPM(14, 0);
  } else {
    state.setMotorRPM(11, 0);
    state.setMotorRPM(12, 0);
    state.setMotorRPM(13, 0);
    state.setMotorRPM(14, 0);
  }
}

void matchLoop() {

}

int main() {
  setup();
  while(1) {
    uint32_t currentTime = micros();
    state.deltaTime = (currentTime - programTime) / 1000.0;
    programTime = currentTime;

    /* Read sensors */
    state.receiver.read();
    state.imu.read_lsm6dsox_gyro();
    state.ref.read_serial();

    state.rmCAN.read_can(CANBUS_1);
    state.rmCAN.read_can(CANBUS_2);

    /* Generate control output */
    if (state.receiver.out.l_switch == 3) {
    state.robotMode = DEMO;
    demoLoop();
    } else if (state.receiver.out.l_switch == 2) {
    state.robotMode = MATCH;
    matchLoop();
    } else {
    state.robotMode = OFF;
    state.motorMap.allOff();
    }

    int index = 11;
    int can_bus = state.rmCAN.motor_arr[index].can_bus;
    int msg_type = state.rmCAN.motor_arr[index].message_type;
    int msg_offset = state.rmCAN.motor_arr[index].message_offset;
    Serial.print(state.rmCAN.output[can_bus][msg_type].buf[msg_offset], BIN);
    Serial.print(" ");
    Serial.println(state.rmCAN.output[can_bus][msg_type].buf[msg_offset + 1], BIN);

    /* Send CAN output */
    state.rmCAN.write_can();

    //Serial.println(state.deltaTime);

    while (micros() - programTime < loopFrequency) continue;

  }
}