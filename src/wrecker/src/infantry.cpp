#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "state/state.h"
#include "state/config.h"

#include "drivers/dr16.h"
#include "drivers/ref_system.h"
#include "drivers/serial_interface.h"

#include "subsystems/gimbal.h"
#include "subsystems/shooter.h"
#include "subsystems/swerveChassis.h"
#include "subsystems/xDrive.h"

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
CAN_message_t canRecieveMessages[3][11];
CAN_message_t tempMessage;
int CANTimer = 0;

// Loop timing
unsigned long deltaT = 5000;
unsigned long lastTime = 0;
unsigned long dumpRate = 2000000; // 2 sec
IntervalTimer serialDumpTmr;

// State
S_Robot robot_state;
C_Robot robot_config;

// Devices
dr16 reciever;
Ref_System refSys;

// Subsystems
Gimbal gimbal;
Shooter shooter;
SwerveChassis swerveChassis;
XDrive xDrive(&robot_config, &robot_state);

flywheel fw_1;
flywheel fw_2;

int t0;

void dump(){
  dump_Robot(&robot_config, &robot_state);
}

// Runs once
void setup() {
  Serial.begin(1000000);
  delay(1000);
  Serial.println("-- INFANTRY ROBOT START --");

  // Hardware setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  t0 = micros();
  pinMode(29, OUTPUT);
  digitalWrite(29, HIGH);

  can1.begin();
  can2.begin();

  can1.setBaudRate(1000000);
  can2.setBaudRate(1000000);

  // serialDumpTmr.priority(0); // Set interval timer to handle serial reads
  // serialDumpTmr.begin(dump, dumpRate);

  // Configure State
  robot_state.robot = 3;

  // Configure subsystems

  // GIMBAL
  robot_config.gimbal.yawPos.K[0] = 2.0;
  robot_config.gimbal.yawPos.K[2] = 0.01;

  robot_config.gimbal.yawVel.Ymin = -150.0;
  robot_config.gimbal.yawVel.Ymax = 150.0;
  robot_config.gimbal.yawVel.K[0] = 0.06;

  robot_config.gimbal.pitchPos.K[0] = 3;
  robot_config.gimbal.pitchPos.K[2] = 0.0;

  robot_config.gimbal.pitchVel.K[0] = 0.014;

  robot_config.gimbal.pitchMax = 50.0;
  robot_config.gimbal.pitchMin = -18.0;
  robot_config.gimbal.pitchOffset = 177;
  robot_config.gimbal.yawOffset = 119.28;

  // CHASSIS

  // FR
  robot_config.swerveChassis.FR.cornerID = 0;
  robot_config.swerveChassis.FR.steerMotorID = 10;
  robot_config.swerveChassis.FR.steerEncoderID = 1 + 1;
  robot_config.swerveChassis.FR.driveMotorID = 7;//8;
  robot_config.swerveChassis.FR.absolute_offset = 45;
  int fr_alignment[9] = {20, 60, 100, 140, 181, 221, 261, 302, 341};
  for (int i = 0; i < 9; i++)
    robot_config.swerveChassis.FR.alignment[i] = fr_alignment[i];

  // FL
  robot_config.swerveChassis.FL.cornerID = 1;
  robot_config.swerveChassis.FL.steerMotorID = 10;
  robot_config.swerveChassis.FL.steerEncoderID = 1 + 2;
  robot_config.swerveChassis.FL.driveMotorID = 10;//4;
  robot_config.swerveChassis.FL.absolute_offset = -45;
  int fl_alignment[9] = {2, 42, 85, 125, 166, 206, 246, 288, 327};
  for (int i = 0; i < 9; i++)
    robot_config.swerveChassis.FL.alignment[i] = fl_alignment[i];

  // BL
  robot_config.swerveChassis.RL.cornerID = 2;
  robot_config.swerveChassis.RL.steerMotorID = 10;
  robot_config.swerveChassis.RL.steerEncoderID = 1 + 3;
  robot_config.swerveChassis.RL.driveMotorID = 10;//2;
  robot_config.swerveChassis.RL.absolute_offset = -135;
  int bl_alignment[9] = {23, 63, 102, 142, 182, 222, 261, 301, 340};
  for (int i = 0; i < 9; i++)
    robot_config.swerveChassis.RL.alignment[i] = bl_alignment[i];

  // BR
  robot_config.swerveChassis.RR.cornerID = 3;
  robot_config.swerveChassis.RR.steerMotorID = 10;
  robot_config.swerveChassis.RR.steerEncoderID = 1 + 4;
  robot_config.swerveChassis.RR.driveMotorID = 10;
  robot_config.swerveChassis.RR.absolute_offset = 135;
  int br_alignment[9] = {38, 77, 119, 157, 199, 240, 280, 321, 360};
  for (int i = 0; i < 9; i++)
    robot_config.swerveChassis.RR.alignment[i] = br_alignment[i];


  // Driver setup
  refSys.init(&robot_state.refSystem);
  reciever.init(&robot_state.driverInput);

  // Subsystem setup
  gimbal.setup(&robot_config.gimbal, &robot_state);
  swerveChassis.setup(&robot_config.swerveChassis, &robot_state);
  shooter.setup(&robot_config.shooter17, &robot_state);

  //fw_1.init(28);
  //fw_2.init(29);
}


// Runs continuously
void loop() {
  // Necesary for motors to recieve data over CAN
  while (can1.read(tempMessage))
    canRecieveMessages[0][tempMessage.id - 0x201] = tempMessage;
  while (can2.read(tempMessage))
    canRecieveMessages[1][tempMessage.id - 0x201] = tempMessage;

  refSys.read_serial();

  // if (Serial.available() > 0)
  //   serial_event(&robot_config, &robot_state);

  // Update devices
  reciever.update();

  // Update subsystems
  //gimbal.update(deltaT);
  // Serial.println();
  // Serial.print(t0);
  Serial.print(", ");
  
  swerveChassis.update(deltaT); //m3508
  shooter.update(deltaT); //m2006
  //Serial.println(deltaT);
  //xDrive.update(deltaT);
  //Serial.println(robot_state);


//58% power gave a high velocity of of 14.9 and a low of 12.9.
//55% power we are fairly confident will never go over if you want but we also are reasonably confident that 58% should work



  // Send CAN every 5000 microseconds
  CANTimer += deltaT;
  if (CANTimer >= 5000) {
    sendC6x0();
    sendGM6020();
    CANTimer = 0;
  }


  // Loop manager: keep this at the bottom. DO NOT MODIFY.
  deltaT = micros() - lastTime;

  while (deltaT < 1000) // 1 ms
    deltaT = micros() - lastTime;
  
  lastTime = micros();
}