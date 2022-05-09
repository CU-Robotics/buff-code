byte buf[14];

uint16_t temp;
int temp2;

void setup() {
  Serial.begin(115200); //usb serial
  Serial5.begin(115200, SERIAL_8E1_RXINV_TXINV);  //reciever serial
  Serial.println("Reciever serial test");

}

void loop() {
  if(Serial5.available() >= 14) {
//    Serial.println(Serial5.available());
    //parse packet
    Serial5.readBytes(buf, 14);
/////////////////////////////////////////
//    for(int i = 0; i < 14; i++) {
////      Serial.print(buf[i], BIN);
//      for(int j = 0; j < 7; j++) {
//        Serial.print(bitRead(buf[i], j));
//      }
//    }

////////////////////////////
//    for(int i = 0; i < 2; i++) {
//      for(int j = 0; j < 7; j++) {
//        Serial.print(bitRead(buf[i], j));
//      }
//    }

/////////////////////////
    temp2 = buf[1] & 0b00000111;
    temp2 = temp2 << 8;
    temp2 = temp2 | buf[0];

    Serial.println(temp2);

//    temp = buf[13];
//    temp = temp << 8;
//    temp = temp || buf[12];
//    Serial.println(bitRead(buf[5], 3), BIN);

  ////////////////
  delay(100);
    while(Serial5.available()) {
      Serial5.read();
    }
  }

}
