/* FreqMeasureMulti - Example with serial output
   http://www.pjrc.com/teensy/td_libs_FreqMeasure.html

   This example code is in the public domain.
*/
#include <FreqMeasureMulti.h>
#include <math.h>

// Measure 3 frequencies at the same time! :-)
FreqMeasureMulti freq1;

// Try different values for which type of measure:
#define MEASURE_PIN 9
#define MEASURE_TYPE FREQMEASUREMULTI_MARK_ONLY
// #define MEASURE_TYPE FREQMEASUREMULTI_RAISING

void setup() {
  Serial.begin(57600);
  while (!Serial) ; // wait for Arduino Serial Monitor
  // analogWriteResolution(10);
  // analogWriteFrequency(PWM_OUT_PIN, 60);
  // analogWrite(PWM_OUT_PIN, 50);

  pinMode(32, OUTPUT);

  delay(10);
  Serial.println("FreqMeasureMulti Begin");
  delay(10);
  freq1.begin(MEASURE_PIN, MEASURE_TYPE);
}

float sum1 = 0;
int count1 = 0;
int freq1Reads[4];
uint8_t levels[4];
elapsedMillis timeout;
uint32_t cycles = 0;
float angle = 0;
float dutyCycle = 0;

void loop() {
  delayMicroseconds(10);
  digitalWrite(32, HIGH);
  delayMicroseconds(10);
  digitalWrite(32, LOW);

  if (freq1.available()) {
    cycles = freq1.read();
  }
  if (timeout > 500) {
    dutyCycle = round(freq1.countToNanoseconds(cycles)/1000);
    angle = map(dutyCycle, 1, 1024, 0, 360);
    Serial.print("count: ");
    Serial.print(count1);
    Serial.print(", cycles: ");
    Serial.print(cycles);
    Serial.print(", duty cycle (ns): ");
    Serial.print(dutyCycle);
    Serial.print(", angle: ");
    Serial.println(angle);
    timeout = 0;
  }
}