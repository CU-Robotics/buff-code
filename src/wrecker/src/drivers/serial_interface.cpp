#include <Arduino.h>

#include "serial_interface.h"


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
      m = Serial.read();

      pid->Imin = Serial.parseFloat();
      pid->Imax = Serial.parseFloat();
      break;

    case 'Y':
      pid->Ymin = Serial.parseFloat();
      pid->Ymax = Serial.parseFloat();

    case 'C':
      pid->continuous = !pid->continuous;
  }  
}

void PID_serial_event(S_PID* pid)
{
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'R':
      pid->R = Serial.parseFloat();
      break;

    case 'Y':
      pid->Y = Serial.parseFloat();
      break;
  } 
}

void dump_PID(C_PID* pid, String ID)
{
  Serial.print(ID); Serial.print("Kp: ");
  Serial.println(pid->K[0]); 

  Serial.print(ID); Serial.print("Ki: "); 
  Serial.println(pid->K[1]); 

  Serial.print(ID); Serial.print("Kd: ");
  Serial.println(pid->K[2]); 

  Serial.print(ID); Serial.print("I: "); 
  Serial.print(pid->Imin); Serial.print(","); Serial.println(pid->Imax); 

  Serial.print(ID); Serial.print("Y: "); 
  Serial.print(pid->Ymin); Serial.print(","); Serial.println(pid->Ymax); 

  Serial.print(ID); Serial.print("C: "); 
  Serial.println(pid->continuous);
}

void dump_PID(S_PID* pid, String ID)
{
  Serial.print(ID); Serial.print("_X: "); 
  Serial.print(pid->X[0]); Serial.print(","); 
  Serial.print(pid->X[1]); Serial.print(",");
  Serial.println(pid->X[2]); 

  Serial.print(ID); Serial.print("_Y: ");
  Serial.println(pid->Y);

  Serial.print(ID); Serial.print("_R: ");
  Serial.println(pid->R);
}

void SwerveModule_serial_event(C_SwerveModule* sm_config){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'M':
      sm_config->moduleID = Serial.parseInt();
      break;

    case 'R':
      sm_config->driveMotorID = Serial.parseInt();
      break;

    case 'T':
      sm_config->steerMotorID = Serial.parseInt();
      break;

    case 'E':
      sm_config->steerEncoderID = Serial.parseInt();
      break;

    case 'A':
      sm_config->alignment[0] = Serial.parseInt();
      sm_config->alignment[1] = Serial.parseInt();
      sm_config->alignment[2] = Serial.parseInt();
      sm_config->alignment[3] = Serial.parseInt();
      sm_config->alignment[4] = Serial.parseInt();
      sm_config->alignment[5] = Serial.parseInt();
      sm_config->alignment[6] = Serial.parseInt();
      sm_config->alignment[7] = Serial.parseInt();
      sm_config->alignment[8] = Serial.parseInt();
      break;

    case 'P':
      PID_serial_event(&sm_config->steerPos);
      break;

    case 'V':
      PID_serial_event(&sm_config->steerVel);
      break;

    case 'D':
      PID_serial_event(&sm_config->driveVel);
      break;
  }
}

void dump_Swerve(C_SwerveModule* sm, String ID){
  Serial.print(ID); Serial.print("M: ");
  Serial.println(sm->moduleID); 

  Serial.print(ID); Serial.print("R: ");
  Serial.println(sm->driveMotorID);

  Serial.print(ID); Serial.print("T: ");
  Serial.println(sm->steerMotorID);

  Serial.print(ID); Serial.print("E: ");
  Serial.println(sm->steerEncoderID);

  Serial.print(ID); Serial.print("A: ");
  Serial.print(sm->alignment[0]); Serial.print(",");
  Serial.print(sm->alignment[1]); Serial.print(",");
  Serial.print(sm->alignment[2]); Serial.print(",");
  Serial.print(sm->alignment[3]); Serial.print(",");
  Serial.print(sm->alignment[4]); Serial.print(",");
  Serial.print(sm->alignment[5]); Serial.print(",");
  Serial.print(sm->alignment[6]); Serial.print(",");
  Serial.print(sm->alignment[7]); Serial.print(",");
  Serial.println(sm->alignment[8]);

  dump_PID(&sm->steerPos, ID + "P");
  dump_PID(&sm->steerVel, ID + "V");
  dump_PID(&sm->driveVel, ID + "D");
}

void dump_Swerve(S_SwerveModule* sm, String ID){
  Serial.print(ID); Serial.print("_sm_state:");
  Serial.print(sm->steer_angle); Serial.print(",");
  Serial.print(sm->steer_speed); Serial.print(",");
  Serial.print(sm->drive_speed); Serial.print(",");
  Serial.println(sm->drive_accel);

  dump_PID(&sm->steerPos, ID + "_sm_steer_pos_pid");
  dump_PID(&sm->steerVel, ID + "_sm_steer_vel_pid");
  dump_PID(&sm->driveVel, ID + "_sm_drive_vel_pid");
}

void SwerveChassis_serial_event(C_SwerveChassis* sc){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'W':
      sc->baseWidth = Serial.parseFloat();
      break;

    case 'L':
      sc->baseLength = Serial.parseFloat();

    case 'I':
      m = Serial.parseInt();
      sc->currentLimit[m] = Serial.parseFloat();
      break;

    case 'A':
      SwerveModule_serial_event(&sc->FL);
      break;

    case 'B':
      SwerveModule_serial_event(&sc->FR);
      break;

    case 'C':
      SwerveModule_serial_event(&sc->RR);
      break;

    case 'D':
      SwerveModule_serial_event(&sc->RL);
      break;
  } 
}

void RailChassis_serial_event(C_RailChassis* rc){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'N':
      m = Serial.parseInt();
      rc->nodes[m] = Serial.parseFloat();
      break;

    case 'P':
      PID_serial_event(&rc->drivePos);
      break;

    case 'V':
      PID_serial_event(&rc->driveVel);
      break;
  } 
}

void dump_Chassis(S_Chassis* ch){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("/chassis_state:"); 
  Serial.print(ch->heading); Serial.print(","); 
  Serial.print(ch->rpm); Serial.print(","); 
  Serial.print(ch->alpha); Serial.print(","); 
  Serial.print(ch->a[0]); Serial.print(","); 
  Serial.println(ch->a[1]);

  dump_Swerve(&ch->FL, "/fl");
  dump_Swerve(&ch->FR, "/fr");
  dump_Swerve(&ch->RR, "/rr");
  dump_Swerve(&ch->RL, "/rl");
}

void dump_RailChassis(C_RailChassis* rc){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("/rail_chassis_config: "); 
  Serial.print(rc->nodes[0]); Serial.print(",");
  Serial.print(rc->nodes[1]); Serial.print(",");
  Serial.print(rc->nodes[2]); Serial.print(",");
  Serial.print(rc->nodes[3]); Serial.print(",");
  Serial.print(rc->nodes[4]); Serial.print(",");
  Serial.print(rc->nodes[5]); Serial.print(",");
  Serial.print(rc->nodes[6]); Serial.print(",");
  Serial.print(rc->nodes[7]); Serial.print(",");
  Serial.print(rc->nodes[8]); Serial.print(",");
  Serial.println(rc->nodes[9]);

  dump_PID(&rc->drivePos, "/rc_drive_pos_pid");
  dump_PID(&rc->driveVel, "/rc_drive_vel_pid");
}

void dump_SwerveChassis(C_SwerveChassis* sc){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("@SW: "); Serial.println(sc->baseWidth);
  Serial.print("@SL: "); Serial.println(sc->baseLength);

  Serial.print("@SI: ");
  Serial.print(sc->currentLimit[0]); Serial.print(","); 
  Serial.print(sc->currentLimit[1]); Serial.print(",");
  Serial.print(sc->currentLimit[2]); Serial.print(",");
  Serial.println(sc->currentLimit[3]);

  dump_Swerve(&sc->FR, "@SA");
  dump_Swerve(&sc->FL, "@SB");
  dump_Swerve(&sc->RL, "@SC");
  dump_Swerve(&sc->RR, "@SD");
}

void Gimbal_serial_event(C_Gimbal* gm){
	char cmd = Serial.read();
  switch (cmd)
  {
    case 'S':
      gm->sensitivity = Serial.parseFloat();
      break;

    case '1':
      gm->pitchOffset = Serial.parseInt();

    case '2':
      gm->yawOffset = Serial.parseInt();

    case 'Y':
      PID_serial_event(&gm->yaw_PID);
      break;

    case 'P':
      PID_serial_event(&gm->pitch_PID);
      break;
  }
}

void Gimbal_serial_event(S_Gimbal* gm){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'Y':
      PID_serial_event(&gm->yaw_PID);
      break;

    case 'P':
      PID_serial_event(&gm->pitch_PID);
      break;
  }
}

void dump_Gimbal(S_Gimbal* gm){
  Serial.print("/gimbal_state: "); 
  Serial.print(gm->yaw); Serial.print(","); 
  Serial.print(gm->pitch); Serial.print(",");
  Serial.println(gm->yawGlobal);

  dump_PID(&gm->yaw_PID, "/gimbal_yaw_pid");
  dump_PID(&gm->pitch_PID, "/gimbal_pitch_pid");
}

void dump_Gimbal(C_Gimbal* gm){
  Serial.print("@BS: "); 
  Serial.println(gm->sensitivity); 

  Serial.print("@B1: "); 
  Serial.println(gm->yawOffset); 

  Serial.print("@B2: ");
  Serial.println(gm->pitchOffset);

  dump_PID(&gm->yaw_PID, "@BY");
  dump_PID(&gm->pitch_PID, "@BP");
}

void Shooter17_serial_event(C_Shooter17* sh){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'L':
      sh->feedRPMLow = Serial.parseInt();
      break;

    case 'H':
      sh->feedRPMHigh = Serial.parseInt();
      break;

    case 'B':
      sh->feedRPMBurst = Serial.parseInt();
      break;

    case 'P':
      m = Serial.parseInt();
      sh->flywheelPower[m] = Serial.parseFloat();
      break;
  }
}

void Shooter42_serial_event(C_Shooter42* sh){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'T':
      sh->feedTimeout = Serial.parseInt();
      break;

    case 'P':
      m = Serial.parseInt();
      sh->flywheelPower[m] = Serial.parseFloat();
      break;
  }
}

void dump_Shooter_State(S_Shooter* sh_state, String ID){
  Serial.print(ID); Serial.print("_firing:"); Serial.println(sh_state->firing);
}

void DriverInput_serial_event(DriverInput*){

}

void dump_DriverInput(DriverInput* di){
  Serial.print("/Driver"); Serial.print("_leftStick:"); 
  Serial.print(di->leftStickX); Serial.print(","); 
  Serial.println(di->leftStickY);
  
  Serial.print("/Driver"); Serial.print("_rightStick:"); 
  Serial.print(di->rightStickX); Serial.print(","); 
  Serial.println(di->rightStickY);
  
  Serial.print("/Driver"); Serial.print("_switch:"); 
  Serial.print(di->leftSwitch); Serial.print(","); 
  Serial.println(di->rightSwitch);
  
  Serial.print("/Driver"); Serial.print("_mouse:"); 
  Serial.print(di->mouseX); Serial.print(","); 
  Serial.println(di->mouseY);
}

void dump_RefSystem_State(S_RefSystem* rf){

}

void serial_event(S_Robot* r_state, C_Robot* r_config){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'G':
      Gimbal_serial_event(&r_config->gimbal);
      break;

    case '7':
      Shooter17_serial_event(&r_config->shooter17);
      break;

    case '4':
      Shooter42_serial_event(&r_config->shooter42);
      break;

    case 'R':
      RailChassis_serial_event(&r_config->railChassis);
      break;

    case 'S':
      SwerveChassis_serial_event(&r_config->swerveChassis);
      break;

    case 'B':
      Gimbal_serial_event(&r_state->gimbal);
  }
}

void dump_Robot(S_Robot* r_state, C_Robot* r_config){
  dump_Chassis(&r_state->chassis);
  // dump_RailChassis(r_config->railChassis);
  dump_SwerveChassis(&r_config->swerveChassis);

  // dump_Gimbal(&r_state->gimbal);
  // dump_Gimbal(&r_config->gimbal);

  // dump_RefSystem_State(r_state->refSystem);

  // dump_DriverInput(r_state->driverInput);
}


