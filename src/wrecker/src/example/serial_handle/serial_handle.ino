/* serial handle sample. this is the teensy side (duh)
 *  This example will read an input msg and reply with
 *  a coordinated response.
*/
#define WHOAMI "Teensy4.1"

float simVal = 0.0;
float phaseScalar = 0.0;
float periodScalar = 0.2;
float amplitudeScalar = 1;

unsigned long loopDur;
unsigned long loopStart;
unsigned long dumpRate = 0.0;
unsigned long cycleSim = 1.0;                                   // this needs to stay under 1 period (best is 0.2 * period)
unsigned long simStart = 0.0;
unsigned long cycleBig = 10.0;
unsigned long last_dump = 0.0;
unsigned long last_update = 0.0;


void updateSim() {
  /*
   * This function is used to demonstrate setting run params
   * on the teensy. (periodScalar will get updated)
   */
  unsigned long t = millis();                                     // get new time

  float dur = (float)(t - simStart);                            // get duration since start of sim
  simVal += amplitudeScalar * cos((dur * periodScalar) + phaseScalar);  // update simVal, dx = b * cos(a * t) where a determines the period,
                                                                //   t is the time alive and b is an amplitude scalar.
}

void serialDump() {
  Serial.print('@'); Serial.print(WHOAMI);
  Serial.print(','); Serial.print(simVal);
  Serial.print(','); Serial.print(phaseScalar);
  Serial.print(','); Serial.print(periodScalar);
  Serial.print(','); Serial.print(amplitudeScalar);
  Serial.print(','); Serial.print(loopDur);
  Serial.print(','); Serial.print(cycleSim);
  Serial.print(','); Serial.print(simStart);
  Serial.print(','); Serial.print(cycleBig);
  Serial.print(','); Serial.println(last_update);
  Serial.print(','); Serial.println(last_dump);
}

void verbose_serialDump() {
  Serial.print('@'); Serial.print(WHOAMI);
  Serial.print(",\n\tsimVal="); Serial.print(simVal);
  Serial.print(",\n\tphaseScale="); Serial.print(phaseScalar);
  Serial.print(",\n\tperiodScale="); Serial.print(periodScalar);
  Serial.print(",\n\tamplitudeScale="); Serial.print(amplitudeScalar);
  Serial.print(",\n\tloopDur="); Serial.print(loopDur);
  Serial.print("us,\n\tcycleSim="); Serial.print(cycleSim);
  Serial.print("ms,\n\tsimStart="); Serial.print(simStart);
  Serial.print("ms,\n\tcycleBig="); Serial.print(cycleBig);
  Serial.print("us,\n\tlast_update="); Serial.print(last_update);
  Serial.print("ms,\n\tlast_dump="); Serial.print(last_dump);
  Serial.print("ms,\n\tloopStart="); Serial.print(loopStart);
  Serial.println("ms");
}

void parse_cmd(char incomingByte) {
  /*
   * We detected a command and will parse it here
   * 0 will reset the sim
   * 1 will set the periodScale
   * 2 will set the amplitudeScale
   * 3 will set the phase shift
   * d will dump the state
   * should switch this to some kind of lookup
   */
  if (incomingByte == '1')
    periodScalar = Serial.parseFloat();
  else if (incomingByte == '2')
    amplitudeScalar = Serial.parseFloat();
  else if (incomingByte == '3')
    phaseScalar = Serial.parseFloat();
  else if (incomingByte == 'd')
    serialDump();
  else if (incomingByte == 'D')
    verbose_serialDump();
  else if (incomingByte == 'r')
    dumpRate = Serial.parseFloat();
  else if (incomingByte == '0'){
    simStart = millis();
    simVal = 0.0;
  }
}

void serial_event() {
  /*
   * this will handle all serial processing.
   * - look for message start
   * - drop invalid bytes
   */
  char incomingByte;
  int byteCount = 0;
  
  byteCount++;
  incomingByte = Serial.read();
  while (Serial.available()) {
    if (incomingByte == '@')                                        // if start char is read start parsing message
      parse_cmd(Serial.read());
  }
}

void setup() {
  Serial.begin(9600);
  simStart = millis();
}

void loop() {
  loopStart = micros();

  if ((loopStart / 100) - last_update > cycleSim) {               // sorta like a debounce but prevents repeats of t 
    updateSim();                                                  // sim runs on the ordr of milli and the rest on micro
    last_update = loopStart;
  }

  if ((loopStart / 100) - last_dump > dumpRate && dumpRate > 0) { // The rates are a minimum duration of time (wait to reach this)
    verbose_serialDump(); 
    last_dump = loopStart / 100;
  }
  
  if (Serial.available())
    serial_event();

  loopDur = micros() - loopStart;
  if (loopDur <= cycleBig)                                         // normalize the loop time to help develop timing reqs
    delay(cycleBig - loopDur);
  else {
    Serial.print("loop cycled in longer time than expected: ");
    Serial.println(loopDur / 100);
  }
}
