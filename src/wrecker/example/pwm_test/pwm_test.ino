// #include <pid.h>

const int outputPin = 29;

int newValue = 0;
int currValue = 0;

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(outputPin, 9375000);
  analogWriteResolution(4);
  analogWrite(outputPin, 8);
}

void loop() {
  // if(Serial.available() > 1) {
  //   newValue = Serial.parseInt();
  //   currValue = analogRead(outputPin);
  //   if(newValue >= 0) {
  //     analogWrite(outputPin, newValue);
  //     Serial.print("Success: ");
  //     Serial.println(newValue);
  //   } else {
  //     Serial.print("error: ");
  //     Serial.println(newValue);
  //   }
  // }
  
}
