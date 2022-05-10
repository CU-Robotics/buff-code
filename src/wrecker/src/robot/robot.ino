#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
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

CAN_message_t chassisSendMsg;
CAN_message_t chassisRecMsg;

CAN_message_t superStructureSendMsg;
CAN_message_t superStructureRecMsg;

struct RobotConfig roboConfig;
struct RobotInput roboInput;

PIDController tlPID;

// gimbal gimbal(&roboConfig, &roboInput);

c610Enc motorTR(3, &chassisSendMsg, 2);
c610Enc motorBL(5, &chassisSendMsg, 3);
c610Enc motorBR(4, &chassisSendMsg, 4);
c610Enc motorTL(2, &chassisSendMsg, 5);

String tempStr = "";

float kp = 0;
float ki = 0;
float kd = 0;

float angle = 0;
float newPower = 0;

float setpoint = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  chassisCAN.begin();
  SuperStructureCAN.begin();
  chassisCAN.setBaudRate(1000000);
  SuperStructureCAN.setBaudRate(1000000);
  Serial.begin(9600);
  Serial.println("robot test");

  tlPID.init(kp,ki,kd);

//   gimbal.init(&superStructureSendMsg);
}


void loop() {

  //send serial in x.xxxxx format with that many digits
  if(Serial.available() > 1) {
    tempStr = Serial.readString();
    if(tempStr.charAt(0) == 'p') {
      kp = tempStr.substring(1,8).toFloat();
      Serial.print("new p string: ");
      Serial.println(tempStr.substring(1,8));
      Serial.print("new p: ");
      Serial.println(kp, 5);
      tlPID.init(kp,ki,kd);
    } else if (tempStr.charAt(0) == 'i') {
      ki = tempStr.substring(1,8).toFloat();
      tlPID.init(kp,ki,kd);
    } else if (tempStr.charAt(0) == 'd') {
      kd = tempStr.substring(1,8).toFloat();
      tlPID.init(kp,ki,kd);
    }
  }

  if(chassisCAN.read(chassisRecMsg)) {
    // Serial.println("------------------"); 

    // Serial.println("got chassis message");
    if (chassisRecMsg.id == 0x202)
    {
      motorTL.updateMotor(&chassisRecMsg);

      // Serial.print("motor id: ");
      // Serial.println(chassisRecMsg.id, HEX);
      // Serial.print("Motor angle: ");
      // Serial.println(motorTL.getAngle());
      // Serial.print("Torque: ");
      // Serial.println(motorTL.getTorque());
      // Serial.print("RPM: ");
      // Serial.println(motorTL.getRpm());
    }
  
  }

  deltaT = micros() - lastTime;
  lastTime = micros();

  setpoint += 0.5;
  if (setpoint > 360) {
    setpoint -= 360;
  }

  newPower = tlPID.calculate(motorTL.getAngle(), setpoint, deltaT);
  motorTL.setPower(newPower);
  chassisCAN.write(chassisSendMsg);
  delay(10);
}
