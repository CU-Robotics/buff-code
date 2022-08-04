 /* Basic Raw HID Example
   Teensy can send/receive 64 byte packets with a
   dedicated program running on a PC or Mac.

   You must select Raw HID from the "Tools > USB Type" menu

   Optional: LEDs should be connected to pins 0-7,
   and analog signals to the analog inputs.

   This example code is in the public domain.
*/
#include <Arduino.h>

struct HID_Device {
  byte buffer[64];
};

HID_Device hid;

int send_hid(HID_Device* hid){
  for (int i = 0; i < 64; i++){
    hid->buffer[i] = byte(i);
  }

  return RawHID.send(hid->buffer, 100);
}

void read_hid(HID_Device* hid){
  int n = RawHID.recv(hid->buffer, 0); // 0 timeout = do not wait
  if (n > 0) {
    // the computer sent a message.  Display the bits
    // of the first byte on pin 0 to 7.  Ignore the
    // ignore 63 bytes

  }
  // every 2 seconds, send a packet to the computer
}

IntervalTimer hidtmr;
unsigned long blink_time;
unsigned long cycle_time = 1000;
int blink_val = 0;

void setup() {
  blink_time = millis();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);
  Serial.println("setup done");
}

// RawHID packets are always 64 bytes

int printVal = 0;

void loop() {
  unsigned  long top_time = micros();
  
  if (millis() - blink_time > 250){
    blink_time = millis();
    blink_val = !blink_val;
    digitalWrite(LED_BUILTIN, blink_val);
  }

  Serial.println("Writing");
  send_hid(&hid);
  Serial.println("done");

  while (micros() - top_time < cycle_time){}
}