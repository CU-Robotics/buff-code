/* serial handle sample. this is the teensy side (duh)
 *  This example will read an input msg and reply with
 *  a coordinated response. It will also run a simulation
 *  of summing changes compared with a continous function.
*/
#define WHOAMI "Teensy4.1"

IntervalTimer serialDumpTmr;
IntervalTimer serialReadTmr;

float simVal;                                                     // Sim state is set in setup and on reset
float simErr;
float simdActual;
float simdUpdate;
float simActual;
float phaseScalar = 0.0;                                          // These only update on request (no setup)
float periodScalar = PI / 50;                                      // default period of 100us
float amplitudeScalar = 1.0;

unsigned long loopDur;
unsigned long loopStart;
unsigned long serialRate = 15000.0;                              // rate to read serial (half of what targets will publish at)
unsigned long dumpRate = 1000000.0;                               // dumpRate is set to 1 million us or 1 second
unsigned long simRate = 5.0;                                     // this needs to stay under 1 period (us)
                                                                  //  period = 2pi / periodScalar, if period is less than update
                                                                  //  the updates will be too infrequent to be accurate (lots of change between update)
unsigned long simStart = 0.0;
unsigned long last_dump = 0.0;
unsigned long last_update = 0.0;

unsigned long sim_timer = 0.0;
unsigned long dump_timer = 0.0;
unsigned long serial_timer = 0.0;

void updateSim() {
  /*
   * This function is used to demonstrate setting run params on the teensy. This function will update the variable
   * simVal using its derivative. The function simVal follows is y = c * cos((a * t) + b). Then, dy = -ac * sin((a * t) + b).
   * The derivative tells us at some time t the change is dy. This example shows how using a discrete summation of dy's
   * will create error in our y. The error is calculated with
   * yActual = c * cos((a * T) + b), yEstimate = sum(c * sin((a * t) + b)) for t in 0:T
   * yError = abs(yActual - yEstimate)
   */
  unsigned long t = micros();                                     // get new time

  float sim_run_time = t - simStart;                              // get duration since start of sim
  //float last_update_sim = last_update - simStart;

  simdUpdate = (-periodScalar * amplitudeScalar * sin((sim_run_time * periodScalar) + phaseScalar));  
                                                                  // update simVal, dx = b * sin((a * t) - c) / a 
  float simMeasure = amplitudeScalar * cos((sim_run_time * periodScalar) + phaseScalar);

  simVal += (simdUpdate) * (t - last_update);
  simErr = abs(simMeasure - simVal);
  simdActual = (simMeasure - simActual); 
  simActual = simMeasure;
  last_update = t;
  sim_timer = micros() - t;
  
}

void reset_sim() {
  simVal = 1.0;
  simErr = 0.0;
  simActual = 1.0;
  simdUpdate = 0.0;
  simdActual = 0.0;
  simStart = micros();
  last_update = micros();
}

class SerialPipe {
  private:
    bool autoDump;
  public:
    SerialPipe();
    void dump();
    void parseCMD(char);
};

SerialPipe::SerialPipe(int baudrate, unsigned long dumpRate, unsigned long readRate, bool autoDump) {
  Serial.begin(baudrate);

  autoDump = autoDump;                                              // Default auto dump is off
  
  serialDumpTmr.priority(1);                                     // Set interval timer to handle serial dumps
  serialDumpTmr.begin(verbose_serialDump, dumpRate);

  serialReadTmr.priority(1);                                     // Set interval timer to handle serial reads
  serialReadTmr.begin(serial_event, readRate);
}

void SerialPipe::dump() {
  
}

void SerialPipe::parseCMD(char firstByte){
    /*
   * We detected a command and will parse it here
   * 0 will reset the sim
   * 1 will set the periodScale
   * 2 will set the amplitudeScale
   * 3 will set the phase shift
   * d/D will dump the state
   * r will set an auto-state dump (every n milliseconds)
   * should switch this to some kind of lookup
   */
  if (firstByte == '1')
    periodScalar = Serial.parseFloat();
  else if (firstByte == '2')
    amplitudeScalar = Serial.parseFloat();
  else if (firstByte == '3')
    phaseScalar = Serial.parseFloat();
  else if (firstByte == 'd')
    serialDump();
  else if (firstByte == 'D')
    verbose_serialDump();
  else if (firstByte == 'r')
    autoDump = !autoDump;
  else if (firstByte == '0'){
    reset_sim();
  }
}

void SerialPipe::serial_event() {
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
    if (incomingByte == '@')                                      // if start char is read start parsing message
      parse_cmd(Serial.read());
  }
}

void serialDump() {
  Serial.print('@'); Serial.print(WHOAMI); // 10 bytes
  Serial.print(','); Serial.print(simVal); // 15 bytes
  Serial.print(','); Serial.print(simActual); // 20 bytes
  Serial.print(','); Serial.print(simErr);  // 25 bytes
  Serial.print(','); Serial.print(simdUpdate); // 30 bytes
  Serial.print(','); Serial.print(simdActual); // 35 bytes
  Serial.print(','); Serial.print(phaseScalar); // 40 bytes
  Serial.print(','); Serial.print(periodScalar); // 45 bytes
  Serial.print(','); Serial.print(amplitudeScalar); // 50 bytes
  Serial.print(','); Serial.print(loopDur); // 55 bytes
  Serial.print(','); Serial.print(last_update); // 60 bytes
  Serial.print(','); Serial.print(sim_timer); // 65 bytes
  Serial.print(','); Serial.print(serial_timer); // 70 bytes
  Serial.print(','); Serial.println(loopStart); // 75 bytes
}

void verbose_serialDump() {
  Serial.print('@'); Serial.print(WHOAMI);
  Serial.print(",\n\tsimVal="); Serial.print(simVal, 5);
  Serial.print(",\n\tsimActual="); Serial.print(simActual, 5);
  Serial.print(",\n\tsimErr="); Serial.print(simErr, 5);
  Serial.print(",\n\tsimdUpdate="); Serial.print(simdUpdate, 5);
  Serial.print(",\n\tsimdActual="); Serial.print(simdActual, 5);
  Serial.print(",\n\tphaseScale="); Serial.print(phaseScalar, 5);
  Serial.print(",\n\tperiodScale="); Serial.print(periodScalar, 5);
  Serial.print(",\n\tamplitudeScale="); Serial.print(amplitudeScalar, 5);
  Serial.print(",\n\tloopDur="); Serial.print(loopDur);
  Serial.print("us,\n\tsimStart="); Serial.print(simStart);
  Serial.print("us,\n\tlast_update="); Serial.print(last_update);
  Serial.print("us,\n\tsim_timer="); Serial.print(sim_timer);
  Serial.print("us,\n\tserial_timer="); Serial.print(serial_timer);
  Serial.print("us,\n\tloopStart="); Serial.print(loopStart);
  Serial.println("us");
}

void setup() {
  
  reset_sim();


  //simUpdateTmr.priority(0);
  //simUpdateTmr.begin(updateSim, simRate); 
}

void loop() {
  loopStart = micros();
  updateSim();                                                        // we want to update this as much as possible
  loopDur = micros()  - loopStart;
  
//  if (loopDur <= cycleBig)                                          // normalize the loop time to help develop timing reqs
//    delayMicroseconds(cycleBig - loopDur);
//  else {
//    Serial.print("loop cycled in longer time than expected: ");
//    Serial.print(loopDur); Serial.println("us");
//  }
}
