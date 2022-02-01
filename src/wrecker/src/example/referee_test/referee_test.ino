void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N2);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()) {
    Serial1.println(Serial1.read());
  }

}
