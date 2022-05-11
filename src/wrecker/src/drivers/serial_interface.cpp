#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"


void PID_serial_event(C_PID pid) 
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

void PID_serial_event(S_PID pid)
{
  char cmd = Serial.read();
  char m;
  switch (cmd)
  {
    case 'R':
      pid.R = Serial.parseFloat();
      break;

    case 'Y':
      pid.Y = Serial.parseFloat();
      break;
  } 
}

void dump_PID_Config(C_PID pid, String ID)
{
  Serial.print(ID); Serial.print("_K:"); 
  Serial.print(pid->K[0]); Serial.print(","); 
  Serial.print(pid->K[1]); Serial.print(",");
  Serial.println(pid->K[2]); 

  Serial.print(ID); Serial.print("_Irange:"); 
  Serial.print(pid->Imin); Serial.print(",");
  Serial.println(pid->Imax); 

  Serial.print(ID); Serial.print("_Yrange:"); 
  Serial.print(pid->Ymin); Serial.print(",");
  Serial.println(pid->Ymax); 
}

void dump_PID_State(S_PID pid, String ID)
{
  Serial.print(ID); Serial.print("_X:"); 
  Serial.print(pid->X[0]); Serial.print(","); 
  Serial.print(pid->X[1]); Serial.print(",");
  Serial.println(pid->X[2]); 
  Serial.print(ID); Serial.print("_Y:");
  Serial.println(pid->Y);
  Serial.print(ID); Serial.print("_R:");
  Serial.println(pid->R);
}

void SwerveModule_serial_event(C_SwerveModule sm_config){
  char cmd = Serial.read();
  char m;
  switch (cmd)
  {
    case 'M':
      sm_config.moduleID = Serial.parseInt();
      break;

    case 'P':
      PID_serial_event(sm_config.steerPos)
      break;

    case 'V':
      PID_serial_event(sm_config.steerVel)
      break;

    case 'D':
      PID_serial_event(sm_config.driveVel)
      break;
  } 
}

void dump_Swerve_State(S_SwerveModule sm_state, String ID){
  Serial.print(ID); Serial.print("_swerve_module:");
  Serial.print(sm_state.steer_angle); Serial.print(",");
  Serial.print(sm_state.steer_speed); Serial.print(",");
  Serial.print(sm_state.drive_speed); Serial.print(",");
  Serial.println(sm_state.drive_accel);

  dump_PID_State(sm_state.steerPos, ID + "_swerve_module");
  dump_PID_State(sm_state.steerVel, ID + "_swerve_module");
  dump_PID_State(sm_state.driveVel, ID + "_swerve_module");
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


void dump_Chassis_State(S_Chassis ch_state){
  /* Need to know variables to use rostopic, std.msgs.dict?  */
  Serial.print("/chassis:"); 
  Serial.print(ch_state.heading); Serial.print(","); 
  Serial.print(ch_state.rpm); Serial.print(","); 
  Serial.print(ch_state.alpha); Serial.print(","); 
  Serial.print(ch_state.a[0]); Serial.print(","); 
  Serial.println(ch_state.a[1]);
  dump_Swerve_State(ch_state.FL, "/front_left");
  dump_Swerve_State(ch_state.FR, "/front_right");
  dump_Swerve_State(ch_state.RR, "/rear_right");
  dump_Swerve_State(ch_state.RL, "/rear_left");
}

void dump_Gimbal_State(S_Gimbal g_state){
  Serial.print("/gimbal:"); 
  Serial.print(g_state.yaw); Serial.print(","); 
  Serial.print(g_state.pitch); Serial.print(",");
  Serial.println(g_state.yawGlobal);
  dump_PID_State(g_state.yaw_PID, "/gimbal_yaw_pid");
  dump_PID_State(g_state.pitch_PID, "/gimbal_pitch_pid");
}

void dump_Shooter_State(S_Shooter sh_state, String ID){
  Serial.print(ID); Serial.print("_firing:"); Serial.println(sh_state.firing);
}

void dump_DriverInput(DriverInput di, String ID){
  Serial.print(ID); Serial.print("/leftStick:"); 
  Serial.print(di.leftStickX); Serial.print(","); Serial.println(di.leftStickY);
  Serial.print(ID); Serial.print("/rightStick:"); 
  Serial.print(di.rightStickX); Serial.print(","); Serial.println(di.rightStickY);
  Serial.print(ID); Serial.print("/Switch:"); 
  Serial.print(di.leftSwitch); Serial.print(","); Serial.println(di.rightSwitch);
  Serial.print(ID); Serial.print("/mouse:"); 
  Serial.print(di.mouseX); Serial.print(","); Serial.println(di.mouseY);
}

void dump_Robot_State(S_Robot* r_state){
  dump_Chassis_State(r_state->chassis);
  dump_Gimbal_State(r_state->gimbal);
}


