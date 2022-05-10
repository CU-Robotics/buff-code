#include "controllers.h"
#ifndef DATA_STRUCTURES_H
#include "data_structures.h"
#endif

#define WHOAMI "Teensy4.1"
#define CYCLE_TIME 1

IntervalTimer serialDumpTmr;
IntervalTimer serialReadTmr;

unsigned long start;
unsigned long dumpRate = 100000.0;                               // dumpRate is set to 100,000 us or 0.1 second
unsigned long readRate = 100000.0;                                      // Same as dump

PID pid;
Servo servo = {490, 120, 23, 45.0f, 0.0f, &pid};

BlinkingLED led;

void serial_event() 
{
  char cmd = Serial.read();
  char m;
  switch (cmd)
  {
    case 'K':
      m = Serial.read();
      switch (m)
      {
        case 'p':
          servo.pid->K[0] = Serial.parseFloat();
          break;
        case 'i':
          servo.pid->K[1] = Serial.parseFloat();
          break;
        case 'd':
          servo.pid->K[2] = Serial.parseFloat();
          break;
      }
      break;
    case 'R':
      servo.R = Serial.parseFloat();
  }  
}

void dump()
{
  Serial.print("Servo:");
  Serial.print(servo.R);
  Serial.print(",PID state:");
  Serial.print(servo.pid->X[0]); Serial.print(",");
  Serial.print(servo.pid->X[1]); Serial.print(",");
  Serial.print(servo.pid->X[2]); Serial.print(",Y:");
  Serial.print(servo.pid->Y);
  Serial.print(",BlinkingLED:");
  Serial.println(led.pwm_ctr);
}

void set_servo(Servo* servo, long dt)
{
  PID_Filter(servo->R, servo->pid, dt);
  int duty = map(servo->pid->Y, -servo->theta_max, servo->theta_max, servo->duty_min, servo->duty_max);
  analogWrite(servo->pin, duty);
}

void blinkLED(BlinkingLED* led)
{
  led->pwm_ctr += 1;
  if (led->pwm_ctr >= led->pw_cycle)
  {
    led->pwm_ctr = 0;
    digitalWrite(led->pin, HIGH);
  }
  if (led->pwm_ctr == led->pw_period)
    digitalWrite(led->pin, LOW);
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  serialDumpTmr.priority(1);                                     // Set interval timer to handle serial dumps
  serialDumpTmr.begin(dump, dumpRate);

  serialReadTmr.priority(1);                                     // Set interval timer to handle serial reads
  serialReadTmr.begin(serial_event, readRate);

  analogWriteFrequency(servo.pin, 50);
  analogWriteResolution(12);
  pinMode(servo.pin, OUTPUT);
  pinMode(led.pin, OUTPUT);
}

void loop() 
{
  unsigned long start_t = micros();
  
  set_servo(&servo, CYCLE_TIME);

  blinkLED(&led);
  
  unsigned long loop_t = micros() - start_t;
  if (loop_t < CYCLE_TIME)
    delayMicroseconds(CYCLE_TIME - loop_t);
}
