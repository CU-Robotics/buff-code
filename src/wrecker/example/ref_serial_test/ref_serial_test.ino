byte buf[50];

int bytes_read = 0;

uint16_t data_length = 0;
byte seq;
byte crc_8;
uint16_t command_code = 0;



void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop() {
//  if(Serial1.available() > 6) {
//    Serial.println();
//    Serial.print("Number of bytes: ");
//    Serial.println(Serial1.available());
//    bytes_read = Serial1.readBytes(buf, 7);
////    Serial.print("bytes read: ");
////    Serial.println(bytes_read);
//
//    data_length = buf[2] << 8;
//    data_length = data_length | buf[1];
//    Serial.print("data length: ");
//    Serial.println(data_length);
//    
//    command_code = buf[6] << 8;
//    command_code = command_code | buf[5];
//    Serial.print("Command: ");
//    Serial.println(command_code, HEX);
//  }

  if(Serial1.read() == 0xA5) {
    Serial.print("Available bytes: ");
    Serial.println(Serial1.available());
    while(!Serial1.available()){}
    Serial.print("Available bytes: ");
    Serial.println(Serial1.available());
    data_length = Serial1.read();
//    data_length = data_length << 8;
    while(!Serial1.available()){}
    data_length = data_length | (Serial1.read() << 8);
    while(!Serial1.available()){}
    seq = Serial1.read();
    while(!Serial1.available()){}
    crc_8 = Serial1.read();
    while(Serial1.available() < 2) {}
    command_code = Serial1.read();
    command_code = command_code | (Serial1.read() << 8);

    Serial.print("command_code: ");
    Serial.println(command_code, HEX);

    Serial.print("data_length: ");
    Serial.println(data_length);

    Serial.println();
  } else {
    while(Serial1.available()) {
      Serial1.read();
    }
  }
}
