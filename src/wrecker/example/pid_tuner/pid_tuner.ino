#define WHOAMI "Teensy4.1"

struct pid_packet {
  float Kp = 0.0;
  float Ki = 0.0;
  float Kd = 0.0;
  float R = 0.0;
  float U = 0.0;
  float Y = 0.0;
  float E = 0.0;
};


IntervalTimer serialDumpTmr;
IntervalTimer serialReadTmr;

float dErr = 0.0;
float iErr = 0.0;

pid_packet pid;

unsigned long start;
unsigned long dumpRate = 100000.0;                               // dumpRate is set to 100,000 us or 0.1 second
unsigned long readRate = 100000.0;                                      // Same as dump


void serial_event() 
{
  char cmd = Serial.read();
  if (Serial.available() > 0) 
    Serial.readStringUntil('@');

  cmd = Serial.read();

  switch (cmd)
  {
    case 'P':
      pid.Kp = Serial.parseFloat();
      break;
    case 'I':
      pid.Ki = Serial.parseFloat();
      break;
    case 'D':
      pid.Kd = Serial.parseFloat();
      break;
    case 'X':
      pid.R = 0.0;
      pid.U = 0.0;
      pid.Y = 0.0;
      pid.E = 0.0;
      break;
  }  
}

void dump()
{
  Serial.print("Reference:");
  Serial.print(pid.R);
  Serial.print(",Output:");
  Serial.print(pid.Y);
  Serial.print(",Error:");
  Serial.print(pid.E);
  Serial.print(",Control:");
  Serial.println(pid.U);
}

void setup() {
  // put your setup code here, to run once:

  serialDumpTmr.priority(1);                                     // Set interval timer to handle serial dumps
  serialDumpTmr.begin(dump, dumpRate);

  serialReadTmr.priority(1);                                     // Set interval timer to handle serial reads
  serialReadTmr.begin(serial_event, readRate);

  start = micros();
}

void loop() {

  unsigned long t = micros() - start;

  pid.R = cos(t * PI / 500000);                                  // Set the period of the function to 1,000,000 us
  
  dErr = (pid.R - pid.Y) - pid.E;                                // Use PID to track the reference
  pid.E = pid.R - pid.Y;
  iErr += pid.E;
  pid.U = (pid.E * pid.Kp) + (dErr * pid.Kd) + (iErr * pid.Ki);
  pid.Y += pid.U;

}