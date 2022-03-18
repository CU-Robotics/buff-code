

IntervalTimer serialDumpTmr;
IntervalTimer serialReadTmr;

float Kp = 1.0;

float error;
float control;
float setValue;
float measuredValue;

unsigned long dumpRate = 100000.0;                               // dumpRate is set to 100,000 us or 0.1 second
unsigned long readRate = 100000.0;                                      // Same as dump


void serial_event() 
{
  if (Serial.available() > 0) 
  {
    char cmd = Serial.read();
    if (cmd == 'K')
      Kp = Serial.parseFloat();
    if (cmd == 'T')
      setValue = Serial.parseFloat();
    if (cmd == 'R')
    {
      setValue = 0.0;
      error = 0.0;
      control = 0.0;
      measuredValue = 0.0;
    }
  }  
}

void dump()
{
  Serial.print("T:");
  Serial.print(setValue);
  Serial.print(",M:");
  Serial.print(measuredValue);
  Serial.print(",E:");
  Serial.print(error);
  Serial.print(",U:");
  Serial.println(control);
}

void setup() {
  // put your setup code here, to run once:

  serialDumpTmr.priority(1);                                     // Set interval timer to handle serial dumps
  serialDumpTmr.begin(dump, dumpRate);

  serialReadTmr.priority(1);                                     // Set interval timer to handle serial reads
  serialReadTmr.begin(serial_event, readRate);
}

void loop() {
  // set measured value here
  // measuredValue = xxxxx

  error = setValue - measuredValue;

  control += Kp * error;

}