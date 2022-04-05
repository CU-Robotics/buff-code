#include "c620.h" // change to whatever the motor controller is

CAN_message_t sendMsg;
CAN_message_t recMsg;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
c610Enc controller1 = c610Enc(1, &sendMsg, 6);

int del = 1; // delay length, figure out why this is necesary!!!!!!!!!

const bool printInfo = true;   //set to true to get stats of motors printed to serial

float power = 0;

float currentAngle = 0;
float targetAngle = 180;
float error = 0;
float dError = 0;
float tempError = 0;

float kp = (1/10);
float kd = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(57600);

  Serial.println("Hello from teensy");
}

void loop() {

  // controller1.setPower(.1);

  currentAngle = controller1.getAngle();

  // Serial.print("currentAngle: ");
  // Serial.println(currentAngle);

  // Serial.print("targetAngle: ");
  // Serial.println(targetAngle);

  // error = ((targetAngle - currentAngle) / 360);
  // dError = error - tempError;
  // tempError = error;
  // power = (error * kp) - (dError * kd);
  power = targetAngle - currentAngle;
  power = power / (360 * 8);
  controller1.setPower(power);
  can1.write(sendMsg);
    
  delay(del);
  if(printInfo && can1.read(recMsg)) {

    controller1.updateMotor(&recMsg);
    
    Serial.println("------------------");
    Serial.print("motor id: ");
    Serial.println(recMsg.id, HEX);
    Serial.print("Motor angle: ");
    Serial.println(currentAngle);
    Serial.print("Torque: ");
    Serial.println(controller1.getTorque());
    Serial.print("RPM: ");
    Serial.println(controller1.getRpm());
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("dError: ");
    Serial.println(dError);
    Serial.print("Target angle: ");
    Serial.println(targetAngle);
    Serial.print("Power: ");
    Serial.println(power);
    
  }

  if(Serial.available() > 1) {
    String data = Serial.read();
    targetAngle = Serial.parseFloat();
    // currentAngle = controller1.getAngle();
    // error = targetAngle - currentAngle;
    // controller1.setPower(error / (360 * 10));
    // can1.write(sendMsg);
    Serial.print("new target: ");
    Serial.println(targetAngle);
  }
  
}
