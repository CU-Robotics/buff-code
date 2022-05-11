#include "state/config.h"


void PID_serial_event(C_PID* pid) 
{
  char cmd = Serial.read();
  char m;
  switch (cmd)
  {
    case 'K':
      m = Serial.read();
      switch (m)
      {
        case 'p':
          pid->K[0] = Serial.parseFloat();
          break;

        case 'i':
          pid->K[1] = Serial.parseFloat();
          break;

        case 'd':
          pid->K[2] = Serial.parseFloat();
          break;

      }
      break;

    case 'I':
      pid->Imin = Serial.parseFloat();
      pid->Imax = Serial.parseFloat();
      break;

    case 'Y':
      pid->Ymin = Serial.parseFloat();
      pid->Ymax = Serial.parseFloat();
  }  
}

void dump_PID_config(C_PID* pid, char* ID)
{
  Serial.print(ID); Serial.print("/Kp:"); Serial.print(pid->K[0]); Serial.print(",");
  Serial.print(ID); Serial.print("/Ki:"); Serial.print(pid->K[1]); Serial.print(",");
  Serial.print(ID); Serial.print("/Kd:"); Serial.print(pid->K[2]); Serial.print(",");
  Serial.print(ID); Serial.print("/Imin:"); Serial.print(pid->Imin); Serial.print(",");
  Serial.print(ID); Serial.print("/Imax:"); Serial.print(pid->Imax); Serial.print(",");
  Serial.print(ID); Serial.print("/Ymin:"); Serial.print(pid->Ymin); Serial.print(",");
  Serial.print(ID); Serial.print("/Ymax:"); Serial.println(pid->Ymax);

}

void SwerveModule_serial_event(){

}

void SwerveChassis_serial_event(){

}

void RailChassis_serial_event(){

}

void Gimbal_serial_event(){
	
}

void Shooter17_serial_event(){

}

void Shooter42_serial_event(){

}
