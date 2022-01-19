int newValue = 0;

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(4, 500);
  analogWriteResolution(15);
  analogWrite(4,256);
}

void loop() {
  if(Serial.available() > 1) {
    newValue = Serial.parseInt();
    if(newValue >= 0) {
      analogWrite(4, newValue);
      Serial.print("Success: ");
      Serial.println(newValue);
    } else {
      Serial.print("error: ");
      Serial.println(newValue);
    }
  }
  
}
