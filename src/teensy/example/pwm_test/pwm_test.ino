#include <PWMServo.h>

PWMServo myServo;


void setup() {
  Serial.begin(9600);
  myServo.attach(1, 1050, 1950);
  myServo.write(90);
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
}

void loop() {
  myServo.write(90);
  delay(15);
  Serial.println("test");
}
