#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

#ifndef DR16_H
#include "dr16.h"
#endif

#ifndef STRUCTS_H
#include "structs.h"
#endif

#include "gimbal.h"
#include "c620.h"
#ifndef PIDCONTROLLER_H
#include "PIDController.h"
#endif


unsigned long deltaT = 0; //variable to store time between loops
unsigned long lastTime = 0;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> chassisCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> SuperStructureCAN;

CAN_message_t chassisSendMsg1;
CAN_message_t chassisSendMsg2;
CAN_message_t chassisSendMsg3;
CAN_message_t chassisSendMsg4;

CAN_message_t chassisRecMsg;

CAN_message_t superStructureSendMsg;
CAN_message_t superStructureRecMsg;

struct RobotConfig roboConfig;


// FORTNITE MOVE!
dr16 reciever;
RobotInput inStruct;
// VICTORY ROYALE!


PIDController flPID;
PIDController blPID;
PIDController brPID;
PIDController frPID;

PIDController flPIDVel;
PIDController blPIDVel;
PIDController brPIDVel;
PIDController frPIDVel;

// gimbal gimbal(&roboConfig, &roboInput);

c610Enc motorFL(3, &chassisSendMsg2, 2);
c610Enc motorBL(2, &chassisSendMsg1, 5);
c610Enc motorBR(1, &chassisSendMsg3, 3);
c610Enc motorFR(4, &chassisSendMsg4, 4);

String tempStr = "";

float kp = 0.015;
float ki = 0;
float kd = 0;

float kpVel = 00.00004;
float kiVel = 0;
float kdVel = 0;
float kfVel = 0.0;

float angle = 0;

float setpoint = 180;
float velSetpoint = 6000;


int bl_alignment[9] = {18, 60, 99, 139, 179, 220, 260, 299, 341};
int br_alignment[9] = {20, 60, 100, 140, 181, 221, 261, 302, 341};
int fr_alignment[9] = {23, 63, 102, 142, 182, 222, 261, 301, 340};
int fl_alignment[9] = {2, 42, 85, 125, 166, 206, 246, 288, 327};

int bl_offset = 0;
int br_offset = 0;
int fr_offset = 0;
int fl_offset = 0;

int bl_rollover = 0;
int br_rollover = 0;
int fr_rollover = 0;
int fl_rollover = 0;

float bl_prev = 0;
float br_prev = 0;
float fr_prev = 0;
float fl_prev = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  chassisCAN.begin();
  SuperStructureCAN.begin();
  chassisCAN.setBaudRate(1000000);
  SuperStructureCAN.setBaudRate(1000000);
  //Serial.begin(9600);
  //Serial.println("robot test");

  frPID.init(kp,ki,kd);
  frPID.enableContinuousInput(360);
  fr_rollover = 0;
  fr_prev = 0;
  
  flPID.init(kp,ki,kd);
  flPID.enableContinuousInput(360);
  fl_rollover = 0;
  fl_prev = 0;

  brPID.init(kp,ki,kd);
  brPID.enableContinuousInput(360);
  br_rollover = 0;
  br_prev = 0;

  blPID.init(kp,ki,kd);
  blPID.enableContinuousInput(360);
  bl_rollover = 0;
  bl_prev = 0;

  frPIDVel.init(kpVel,kiVel,kdVel,kfVel);
  flPIDVel.init(kpVel,kiVel,kdVel,kfVel);
  brPIDVel.init(kpVel,kiVel,kdVel,kfVel);
  blPIDVel.init(kpVel,kiVel,kdVel,kfVel);


  // RECIEVER
  reciever.init(&inStruct);
  Serial.begin(115200);
  Serial.println("dr16 test");

  fr_offset = find_match(motorFR.getAngle(), fr_alignment, 9);
  fl_offset = find_match(motorFL.getAngle(), fl_alignment, 9);
  br_offset = find_match(motorBR.getAngle(), br_alignment, 9);
  bl_offset = find_match(motorBL.getAngle(), bl_alignment, 9);
  fr_rollover = 0;
  fl_rollover = 0;
  br_rollover = 0;
  bl_rollover = 0;

  Serial.println("CALIBRATE");

  //debugging
  /*while (Serial5.available())
  {
      Serial5.read();
  }*/

//   gimbal.init(&superStructureSendMsg);
}

int find_match(int curr_value, int* alignment_table, int table_size) {
  int smallest_distance = 360;
  int best_offset = 0;
  for (int i = 0; i < table_size; i++) {
    Serial.println(alignment_table[i]);
    int distance = abs(curr_value - alignment_table[i]);
    if (distance < smallest_distance) {
      smallest_distance = distance;
      best_offset = alignment_table[i];
    }
  }
  return best_offset;
}

void loop() {
  //send serial in x.xxxxx format with that many digits
  /*if(Serial.available() > 1) {
    tempStr = Serial.readString();
    if(tempStr.charAt(0) == 'p') {
      kp = tempStr.substring(1,16).toFloat() / 100.0;
      Serial.print("new p string: ");
      Serial.println(tempStr.substring(1,16));
      Serial.print("new p: "); Serial.println(kp, 5);
      frPID.init(kp,ki,kd);
    } else if (tempStr.charAt(0) == 'i') {
      ki = tempStr.substring(1,16).toFloat() / 100.0;
      frPID.init(kp,ki,kd);
    } else if (tempStr.charAt(0) == 'd') {
      kd = tempStr.substring(1,16).toFloat() / 100.0;
      frPID.init(kp,ki,kd);
    } else if (tempStr.charAt(0) == 'c') {
      Serial.println("CALIBRATE");
      fr_offset = find_match(motorFR.getAngle(), fr_alignment, 9);
      fl_offset = find_match(motorFL.getAngle(), fl_alignment, 9);
      br_offset = find_match(motorBR.getAngle(), br_alignment, 9);
      bl_offset = find_match(motorBL.getAngle(), bl_alignment, 9);
      fr_rollover = 0;
      fl_rollover = 0;
      br_rollover = 0;
      bl_rollover = 0;
    }

    else if(tempStr.charAt(0) == 'j') {
      kpVel = tempStr.substring(1,16).toFloat();
      frPIDVel.init(kpVel,kiVel,kdVel);
    } else if (tempStr.charAt(0) == 'k') {
      kiVel = tempStr.substring(1,16).toFloat();
      frPIDVel.init(kpVel,kiVel,kdVel);
    } else if (tempStr.charAt(0) == 'l') {
      kdVel = tempStr.substring(1,16).toFloat();
      frPIDVel.init(kpVel,kiVel,kdVel);
    } else if (tempStr.charAt(0) == 'm') {
      kfVel = tempStr.substring(1,16).toFloat();
      frPIDVel.init(kpVel,kiVel,kdVel, kfVel);
    }
  }*/

  if (chassisCAN.read(chassisRecMsg)) {
    // Serial.println("------------------"); 

    // Serial.println("got chassis message");
    if (chassisRecMsg.id == 0x204)
    {
      motorFR.updateMotor(&chassisRecMsg);
    }
    if (chassisRecMsg.id == 0x203)
    {
      motorFL.updateMotor(&chassisRecMsg);
    }
    if (chassisRecMsg.id == 0x202)
    {
      motorBL.updateMotor(&chassisRecMsg);
    }
    if (chassisRecMsg.id == 0x201)
    {
      motorBR.updateMotor(&chassisRecMsg);
    }
  }

  deltaT = micros() - lastTime;
  lastTime = micros();


  //Serial.println("Position: " + String(kp, 16) + ", " + String(ki, 16) + ", " + String(kd, 16));
  //Serial.println("Velocity: " + String(kpVel, 16) + ", " + String(kiVel, 16) + ", " + String(kdVel, 16) + ", " + String(kfVel, 16));

  //Serial.println(setpoint);




  //reciever.update();
  bool w = inStruct.w;
  bool a = inStruct.a;
  bool s = inStruct.s;
  bool f = inStruct.f;

  int steer_target = 0;

  int x = 0;
  int y = 0;
  if (w) {
    x += 1;
  }
  if (a) {
    y -= 1;
  }
  if (s) {
    x -= 1;
  }
  if (f) {
    y += 1;
  }

  if (x == 1) {
    if (y == 1) {
      steer_target = 45;
    } else if (y == -1) {
      steer_target = 315;
    } else {
      steer_target = 0;
    }
  } else if (x == -1) {
    if (y == 1) {
      steer_target = 135;
    } else if (y == -1) {
      steer_target = 225;
    } else {
      steer_target = 180;
    }
  } else {
    if (y == 1) {
      steer_target = 90;
    } else if (y == -1) {
      steer_target = 270;
    }
  }

  steer_target = 0;


  // VEL PID
  float fr_rpm = motorFR.getRpm();

  // POS PID
  float fr_raw_pos = motorFR.getAngle();
  float fr_pos = ((fr_raw_pos + fr_rollover * 360) - fr_offset) * (9.0/25.0);

  if ((fr_raw_pos - fr_prev) > 180) {
    fr_rollover--;
    fr_pos = ((fr_raw_pos + fr_rollover * 360) - fr_offset) * (9.0/25.0);
  } else if ((fr_prev - fr_raw_pos) > 180) {
    fr_rollover++;
    fr_pos = ((fr_raw_pos + fr_rollover * 360) - fr_offset) * (9.0/25.0);
  }

  int fr_pos_temp = int(fr_pos * 100);
  fr_pos_temp = fr_pos_temp % 36000;
  fr_pos = fr_pos_temp / 100.0;

  fr_pos -= 45;

  if (fr_pos < 0) {
    fr_pos += 360;
  }


  // VEL PID
  float fl_rpm = motorFL.getRpm();

  // POS PID
  float fl_raw_pos = motorFL.getAngle();
  float fl_pos = ((fl_raw_pos + fl_rollover * 360) - fl_offset) * (9.0/25.0);

  if ((fl_raw_pos - fl_prev) > 180) {
    fl_rollover--;
    fl_pos = ((fl_raw_pos + fl_rollover * 360) - fl_offset) * (9.0/25.0);
  } else if ((fl_prev - fl_raw_pos) > 180) {
    fl_rollover++;
    fl_pos = ((fl_raw_pos + fl_rollover * 360) - fl_offset) * (9.0/25.0);
  }

  int fl_pos_temp = int(fl_pos * 100);
  fl_pos_temp = fl_pos_temp % 36000;
  fl_pos = fl_pos_temp / 100.0;

  fl_pos += 45;

  if (fl_pos < 0) {
    fl_pos += 360;
  }


  // VEL PID
  float br_rpm = motorBR.getRpm();

  // POS PID
  float br_raw_pos = motorBR.getAngle();
  float br_pos = ((br_raw_pos + br_rollover * 360) - br_offset) * (9.0/25.0);

  if ((br_raw_pos - br_prev) > 180) {
    br_rollover--;
    br_pos = ((br_raw_pos + br_rollover * 360) - br_offset) * (9.0/25.0);
  } else if ((br_prev - br_raw_pos) > 180) {
    br_rollover++;
    br_pos = ((br_raw_pos + br_rollover * 360) - br_offset) * (9.0/25.0);
  }

  int br_pos_temp = int(br_pos * 100);
  br_pos_temp = br_pos_temp % 36000;
  br_pos = br_pos_temp / 100.0;

  br_pos -= 135;

  if (br_pos < 0) {
    br_pos += 360;
  }


  // VEL PID
  float bl_rpm = motorBL.getRpm();

  // POS PID
  float bl_raw_pos = motorBL.getAngle();
  float bl_pos = ((bl_raw_pos + bl_rollover * 360) - bl_offset) * (9.0/25.0);

  if ((bl_raw_pos - bl_prev) > 180) {
    bl_rollover--;
    bl_pos = ((bl_raw_pos + bl_rollover * 360) - bl_offset) * (9.0/25.0);
  } else if ((bl_prev - bl_raw_pos) > 180) {
    bl_rollover++;
    bl_pos = ((bl_raw_pos + bl_rollover * 360) - bl_offset) * (9.0/25.0);
  }

  int bl_pos_temp = int(bl_pos * 100);
  bl_pos_temp = bl_pos_temp % 36000;
  bl_pos = bl_pos_temp / 100.0;

  bl_pos += 135;

  if (bl_pos < 0) {
    bl_pos += 360;
  }


  /*Serial.println("---");
  Serial.println(String(fr_raw_pos, 10));
  Serial.println(String(fr_offset, 10));
  Serial.println(String(fr_rollover, 10));*/
  //Serial.println(String(fr_pos, 10));


  // 1
  float newPower = frPID.calculate(fr_pos, steer_target, deltaT);
  float velPower = frPIDVel.calculate(fr_rpm, -newPower * 10000, deltaT);
  motorFR.setPower(velPower);

  chassisCAN.write(chassisSendMsg1);

  fr_prev = fr_raw_pos;


  // 2
  newPower = flPID.calculate(fl_pos, steer_target, deltaT);
  velPower = flPIDVel.calculate(fl_rpm, -newPower * 10000, deltaT);
  motorFL.setPower(velPower);

  chassisCAN.write(chassisSendMsg2);

  fl_prev = fl_raw_pos;


  // 3
  newPower = brPID.calculate(br_pos, steer_target, deltaT);
  velPower = brPIDVel.calculate(br_rpm, -newPower * 10000, deltaT);
  motorBR.setPower(velPower);

  chassisCAN.write(chassisSendMsg3);

  br_prev = br_raw_pos;


  // 4
  newPower = blPID.calculate(bl_pos, steer_target, deltaT);
  velPower = blPIDVel.calculate(bl_rpm, -newPower * 10000, deltaT);
  motorBL.setPower(velPower);

  chassisCAN.write(chassisSendMsg4);

  bl_prev = bl_raw_pos;



  Serial.println(String(fl_pos) + "  -  " + String(fr_pos) + "  -  " + String(bl_pos) + "  -  " + String(br_pos));
  //Serial.println(String(motorFL.getAngle()) + "  -  " + String(motorFR.getAngle()) + "  -  " + String(motorBL.getAngle()) + "  -  " + String(motorBR.getAngle()));

  //motorBR.setPower(-newPower);
  //chassisCAN.write(chassisSemdMsg2);
  delay(1);
}