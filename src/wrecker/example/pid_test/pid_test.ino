#include <PID_v2.h>   //this needs to be installed, probably most easily through the library manager

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; //create an object for CAN bus 1, which is on pin 22 for tx and 23 for rx

short voltage = 0;   //value that will be sent to the motor

PID_v2 anglePID(10, 8, .75, PID::Direct);  //first value is P, Second is I, third is D, fourth vallue does something important I presume

PID_v2 rpmPID(.5, 0, 0, PID::Direct);  //first value is P, Second is I, third is D, fourth vallue does something important I presume


//breaking the voltage value into two discrete bytes
byte bOne = highByte(voltage);
byte bTwo = lowByte(voltage);

//target values
short targetAngle = 4000; //initial target angle in the middle so roll over isn't a big problem
short targetRPM = 6000;

//values to be recieved from motor
int measuredAngle = 0;
short measuredTorque = 0;
short measuredRPM = 0;

CAN_message_t sendMsg;  //message object to store CAN messages that will be sent to motor
CAN_message_t recMsg;   //message object to store CAN messages recieved from motor

void setup() {
  Serial.begin(9600);
  can1.begin(); //initialize the CAN bus
  can1.setBaudRate(1000000);  //the DJI stuff uses 1mbps baudrate

  while(!can1.read(recMsg)){    //wait to recieve data from motor
    delay(1);
  }

  measuredAngle = recMsg.buf[0];  //get the high byte into the variable
  measuredAngle = measuredAngle << 8;   //move the high byte into the high position
  measuredAngle = measuredAngle | recMsg.buf[1];  //get the low byte into the variable

  measuredRPM = recMsg.buf[2];  //get the high byte into the variable
  measuredRPM = measuredRPM << 8;   //move the high byte into the high position
  measuredRPM = measuredRPM | recMsg.buf[3];  //get the low byte into the variable

  anglePID.SetOutputLimits(-10000, 10000);   //currently limited to 10000 for testing
  anglePID.SetSampleTime(10);   //drop sample time down to 10ms from the default 100ms, 1ms seems to make it jittery

  rpmPID.SetOutputLimits(-16000, 16000);   //currently limited to 10000 for testing
  rpmPID.SetSampleTime(10);   //drop sample time down to 10ms from the default 100ms, 1ms seems to make it jittery

  anglePID.Start(measuredAngle, //input
                 0,             //current output
                 targetAngle);  //setpoint
                 
  rpmPID.Start(measuredRPM, //input
                 0,             //current output
                 targetRPM);  //setpoint
}

void loop() {
  if(can1.read(recMsg)) {
    measuredAngle = recMsg.buf[0];  //get second byte and put into firstt byte of measured angle
    measuredAngle = measuredAngle << 8;   //move second byte into second byte of measured angle
    measuredAngle = measuredAngle | recMsg.buf[1];  //stick the second byte into second byte of measured angle

    measuredRPM = recMsg.buf[2];  //get the high byte into the variable
    measuredRPM = measuredRPM << 8;   //move the high byte into the high position
    measuredRPM = measuredRPM | recMsg.buf[3];  //get the low byte into the variable

    voltage = anglePID.Run(measuredAngle);
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.print(", angle: ");
    Serial.print(measuredAngle);
    Serial.print(", rpm: ");
    Serial.println(measuredRPM);
  }

  anglePID.Compute();   //must be run to recalculate the PID output, maybe should be moved to just before anglePID.run()
  
  sendMsg.id = 0x1FF;    //1ff is ID for first 4 GM6020 motors or second 4 C620 ESCs

  byte bOne = highByte(voltage);
  byte bTwo = lowByte(voltage);
  
  sendMsg.buf[0] = bOne; //set high order byte
  sendMsg.buf[1] = bTwo; //set low order byte

  can1.write(sendMsg); //send voltage to motor

  
  delay(5);   //this should be removed eventually, but without it things get weird, better timing methods should be ussed
}
