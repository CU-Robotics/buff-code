#include <Arduino.h>

#include "state/state.h"
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

void dump_PID(S_PID* pid, String ID)
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

void SwerveModule_serial_event(C_SwerveModule* sm_config){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'M':
      sm_config->moduleID = Serial.parseInt();
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
  Serial.print(ID); Serial.print("_swerve_module_config:");
  Serial.print(sm->moduleID); Serial.print(",");
  Serial.print(sm->alignment[0]); Serial.print(",");
  Serial.print(sm->alignment[1]); Serial.print(",");
  Serial.print(sm->alignment[2]); Serial.print(",");
  Serial.print(sm->alignment[3]); Serial.print(",");
  Serial.print(sm->alignment[4]); Serial.print(",");
  Serial.print(sm->alignment[5]); Serial.print(",");
  Serial.print(sm->alignment[6]); Serial.print(",");
  Serial.print(sm->alignment[7]); Serial.print(",");
  Serial.println(sm->alignment[8]);

  dump_PID(&sm->steerPos, ID + "_swerve_module_pid");
  dump_PID(&sm->steerVel, ID + "_swerve_module_pid");
  dump_PID(&sm->driveVel, ID + "_swerve_module_pid");
}

void dump_Swerve(S_SwerveModule* sm, String ID){
  Serial.print(ID); Serial.print("_swerve_module_state:");
  Serial.print(sm->steer_angle); Serial.print(",");
  Serial.print(sm->steer_speed); Serial.print(",");
  Serial.print(sm->drive_speed); Serial.print(",");
  Serial.println(sm->drive_accel);

  dump_PID(&sm->steerPos, ID + "_swerve_module_pid");
  dump_PID(&sm->steerVel, ID + "_swerve_module_pid");
  dump_PID(&sm->driveVel, ID + "_swerve_module_pid");
}


void SwerveChassis_serial_event(C_SwerveChassis* sc){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'W':
      sc->drivebaseWidth = Serial.parseFloat();
      break;

    case 'L':
      sc->drivebaseLength = Serial.parseFloat();

    case '0':
      sc->currentLimitLvl0 = Serial.parseFloat();
      break;

    case '1':
      sc->currentLimitLvl1 = Serial.parseFloat();
      break;

    case '2':
      sc->currentLimitLvl2 = Serial.parseFloat();
      break;

    case '3':
      sc->currentLimitLvl3 = Serial.parseFloat();
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
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'N':
      rc->numNodes = Serial.parseInt();
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

  dump_Swerve(&ch->FL, "/front_left");
  dump_Swerve(&ch->FR, "/front_right");
  dump_Swerve(&ch->RR, "/rear_right");
  dump_Swerve(&ch->RL, "/rear_left");
}

void dump_RailChassis(C_RailChassis* rc){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("/rail_chassis_config:"); 
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

  dump_PID(&rc->drivePos, "/rail_chassis_drive_pos_pid");
  dump_PID(&rc->driveVel, "/rail_chassis_drive_vel_pid");
}

void dump_SwerveChassis(C_SwerveChassis* sc){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("/swerve_chassis_config:"); 
  Serial.print(sc->drivebaseWidth); Serial.print(","); 
  Serial.print(sc->drivebaseLength); Serial.print(","); 
  Serial.print(sc->currentLimitLvl0); Serial.print(","); 
  Serial.print(sc->currentLimitLvl1); Serial.print(",");
  Serial.print(sc->currentLimitLvl2); Serial.print(",");
  Serial.println(sc->currentLimitLvl3);

  dump_Swerve(&sc->FL, "/front_left");
  dump_Swerve(&sc->FR, "/front_right");
  dump_Swerve(&sc->RR, "/rear_right");
  dump_Swerve(&sc->RL, "/rear_left");
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
  Serial.print("/gimbal_state:"); 
  Serial.print(gm->yaw); Serial.print(","); 
  Serial.print(gm->pitch); Serial.print(",");
  Serial.println(gm->yawGlobal);

  dump_PID(&gm->yaw_PID, "/gimbal_yaw_pid");
  dump_PID(&gm->pitch_PID, "/gimbal_pitch_pid");
}

void dump_Gimbal(C_Gimbal* gm){
  Serial.print("/gimbal_config:"); 
  Serial.print(gm->sensitivity); Serial.print(","); 
  Serial.print(gm->yawOffset); Serial.print(",");
  Serial.println(gm->pitchOffset);

  dump_PID(&gm->yaw_PID, "/gimbal_yaw_pid");
  dump_PID(&gm->pitch_PID, "/gimbal_pitch_pid");
}

void Shooter17_serial_event(C_Shooter17* sh){
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

    case '0':
      sh->flywheelPowerLvl0 = Serial.parseFloat();
      break;

    case '1':
      sh->flywheelPowerLvl1 = Serial.parseFloat();
      break;

    case '2':
      sh->flywheelPowerLvl2 = Serial.parseFloat();
      break;

    case '3':
      sh->flywheelPowerLvl3 = Serial.parseFloat();
      break;
  }
}

void Shooter42_serial_event(C_Shooter42* sh){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'T':
      sh->feedTimeout = Serial.parseInt();
      break;

    case '0':
      sh->flywheelPowerLvl0 = Serial.parseFloat();
      break;

    case '1':
      sh->flywheelPowerLvl1 = Serial.parseFloat();
      break;

    case '2':
      sh->flywheelPowerLvl2 = Serial.parseFloat();
      break;

    case '3':
      sh->flywheelPowerLvl3 = Serial.parseFloat();
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
  Serial.print("/ref_sys:"); 
  Serial.print(rf->robotLevel); Serial.print(","); Serial.println(rf->matchTime);
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
  dump_Chassis(r_state->chassis);
  // dump_RailChassis(r_config->railChassis);
  dump_SwerveChassis(r_config->swerveChassis);

  // dump_Gimbal(&r_state->gimbal);
  // dump_Gimbal(&r_config->gimbal);

  // dump_RefSystem_State(r_state->refSystem);

  // dump_DriverInput(r_state->driverInput);
}


