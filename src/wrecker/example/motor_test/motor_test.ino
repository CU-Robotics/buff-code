#include "c620.h" // change to whatever the motor controller is

CAN_message_t sendMsg;
CAN_message_t recMsg;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
c620CAN controller1 = c620CAN(1, &sendMsg);

int del = 500; // delay length

const bool printInfo = true;   //set to true to get stats of motors printed to serial

float power;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.begin(9600);

  Serial.println("Hello from teensy");
}

void loop() {

  // controller1.setPower(.1);

  can1.write(sendMsg);
  
  delay(del);
  if(printInfo && can1.read(recMsg)) {

    controller1.updateMotor(&recMsg);
    
    Serial.println("------------------");
    Serial.print("motor id: ");
    Serial.println(recMsg.id, HEX);
    Serial.print("Motor angle: ");
    Serial.println(controller1.getAngle());
    Serial.print("Torque: ");
    Serial.println(controller1.getTorque());
    Serial.print("RPM: ");
    Serial.println(controller1.getRpm());
    
  }

  if(Serial.available() > 1) {
    String data = Serial.read();
    power = Serial.parseFloat();
    controller1.setPower(power);
    can1.write(sendMsg);
    Serial.print("new power: ");
    Serial.println(power);
  }
  
}
