#include <Arduino.h>

#include "serial_interface.h"


void PID_serial_event(C_PID* config, S_PID* state) 
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
          config->K[0] = Serial.parseFloat();
          break;

        case 'i':
          config->K[1] = Serial.parseFloat();
          break;

        case 'd':
          config->K[2] = Serial.parseFloat();
          break;

      }
      break;

    case 'I':
      m = Serial.read();

      config->Imin = Serial.parseFloat();
      config->Imax = Serial.parseFloat();
      break;

    case 'Y':
      config->Ymin = Serial.parseFloat();
      config->Ymax = Serial.parseFloat();

    case 'R':
      state->R = Serial.parseFloat();
      break;

    case 'C':
      config->continuous = !config->continuous;
  }  
}

void dump_PID(C_PID* pid, String ID)
{
  Serial.print(ID); Serial.print("K: ");
  Serial.print(pid->K[0]); Serial.print(","); 
  Serial.print(pid->K[1]); Serial.print(",");
  Serial.println(pid->K[2]); 

  Serial.print(ID); Serial.print("I: "); 
  Serial.print(pid->Imin); Serial.print(","); Serial.println(pid->Imax); 

  Serial.print(ID); Serial.print("O: "); 
  Serial.print(pid->Ymin); Serial.print(","); Serial.println(pid->Ymax); 

  Serial.print(ID); Serial.print("C: "); 
  Serial.println(pid->continuous);
}

void dump_PID(S_PID* pid, String ID)
{
  Serial.print(ID); Serial.print("X: "); 
  Serial.print(pid->X[0]); Serial.print(","); 
  Serial.print(pid->X[1]); Serial.print(",");
  Serial.println(pid->X[2]); 

  Serial.print(ID); Serial.print("Y: ");
  Serial.println(pid->Y);

  Serial.print(ID); Serial.print("R: ");
  Serial.println(pid->R);
}

void SwerveModule_serial_event(C_SwerveModule* config, S_SwerveModule* state){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'M':
      config->moduleID = Serial.parseInt();
      break;

    case 'C':
      config->cornerID = Serial.parseInt();
      break;

    case 'R':
      config->driveMotorID = Serial.parseInt();
      break;

    case 'T':
      config->steerMotorID = Serial.parseInt();
      break;

    case 'E':
      config->steerEncoderID = Serial.parseInt();
      break;

    case 'A':
      config->alignment[0] = Serial.parseInt();
      config->alignment[1] = Serial.parseInt();
      config->alignment[2] = Serial.parseInt();
      config->alignment[3] = Serial.parseInt();
      config->alignment[4] = Serial.parseInt();
      config->alignment[5] = Serial.parseInt();
      config->alignment[6] = Serial.parseInt();
      config->alignment[7] = Serial.parseInt();
      config->alignment[8] = Serial.parseInt();
      break;

    case 'P':
      PID_serial_event(&config->steerPos, &state->steerPos);
      break;

    case 'V':
      PID_serial_event(&config->steerVel, &state->steerVel);
      break;

    case 'D':
      PID_serial_event(&config->driveVel, &state->driveVel);
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
  Serial.print(ID); Serial.print("S: ");
  Serial.print(sm->steer_angle); Serial.print(",");
  Serial.print(sm->steer_speed); Serial.print(",");
  Serial.print(sm->drive_speed); Serial.print(",");
  Serial.println(sm->drive_accel);

  dump_PID(&sm->steerPos, ID + "P");
  dump_PID(&sm->steerVel, ID + "V");
  dump_PID(&sm->driveVel, ID + "D");
}

void SwerveChassis_serial_event(C_SwerveChassis* config, S_Chassis* state){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'W':
      config->baseWidth = Serial.parseFloat();
      break;

    case 'L':
      config->baseLength = Serial.parseFloat();

    case 'I':
      m = Serial.parseInt();
      config->currentLimit[m] = Serial.parseFloat();
      break;

    case 'A':
      SwerveModule_serial_event(&config->FR, &state->FR);
      break;

    case 'B':
      SwerveModule_serial_event(&config->FL, &state->FL);
      break;

    case 'C':
      SwerveModule_serial_event(&config->RL, &state->RL);
      break;

    case 'D':
      SwerveModule_serial_event(&config->RR, &state->RR);
      break;
  } 
}

void RailChassis_serial_event(C_RailChassis* config, S_Chassis* state){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'N':
      m = Serial.parseInt();
      config->nodes[m] = Serial.parseFloat();
      break;

    case 'P':
      PID_serial_event(&config->drivePos, &state->drivePos);
      break;

    case 'V':
      PID_serial_event(&config->driveVel, &state->driveVel);
      break;
  } 
}

void dump_RailChassis(C_RailChassis* rc){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("@R"); 
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

  dump_PID(&rc->drivePos, "@RP");
  dump_PID(&rc->driveVel, "@RV");
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

void dump_Chassis(S_Chassis* ch){
  /* Need to know strings to use rostopic, std.msgs.dict?  */
  Serial.print("/SS: "); 
  Serial.print(ch->heading); Serial.print(","); 
  Serial.print(ch->rpm); Serial.print(","); 
  Serial.print(ch->alpha); Serial.print(","); 
  Serial.print(ch->a[0]); Serial.print(","); 
  Serial.println(ch->a[1]);

  dump_Swerve(&ch->FL, "/SA");
  dump_Swerve(&ch->FR, "/SB");
  dump_Swerve(&ch->RR, "/SC");
  dump_Swerve(&ch->RL, "/SD");
}

void dump_Gimbal(S_Gimbal* gm){
  Serial.print("/GT: "); 
  Serial.print(gm->yaw); Serial.print(","); 
  Serial.print(gm->pitch); Serial.print(",");
  Serial.print(gm->yaw_reference); Serial.print(",");
  Serial.print(gm->pitch_reference); Serial.print(",");

  Serial.println(gm->yawGlobal);

  dump_PID(&gm->yawVel, "/GY");
  dump_PID(&gm->pitchVel, "/GP");
}

void dump_Gimbal(C_Gimbal* gm){
  Serial.print("@GS: "); 
  Serial.println(gm->sensitivity); 

  Serial.print("@GA: "); 
  Serial.println(gm->yawOffset); 

  Serial.print("@GG: ");
  Serial.println(gm->pitchOffset);

  dump_PID(&gm->yawVel, "@GY");
  dump_PID(&gm->pitchVel, "@GP");
}


void Gimbal_serial_event(C_Gimbal* config, S_Gimbal* state){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'S':
      config->sensitivity = Serial.parseFloat();
      break;

    case 'A':
      config->pitchOffset = Serial.parseInt();

    case 'G':
      config->yawOffset = Serial.parseInt();

    case 'W':
      state->yaw_reference = Serial.parseFloat();
      break;

    case 'H':
      state->pitch_reference = Serial.parseFloat();
      dump_Gimbal(state);
      break;

    case 'Y':
      PID_serial_event(&config->yawVel, &state->yawVel);
      break;

    case 'P':
      PID_serial_event(&config->pitchVel, &state->pitchVel);
      break;
  }
}

void Shooter17_serial_event(C_Shooter17* config, S_Shooter* state){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'L':
      config->feedRPMLow = Serial.parseInt();
      break;

    case 'H':
      config->feedRPMHigh = Serial.parseInt();
      break;

    case 'B':
      config->feedRPMBurst = Serial.parseInt();
      break;

    case 'P':
      m = Serial.parseInt();
      config->flywheelPower[m] = Serial.parseFloat();
      break;
  }
}

void Shooter42_serial_event(C_Shooter42* config, S_Shooter* state){
  int m;
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'T':
      config->feedTimeout = Serial.parseInt();
      break;

    case 'P':
      m = Serial.parseInt();
      config->flywheelPower[m] = Serial.parseFloat();
      break;
  }
}

void dump_Shooter_State(S_Shooter* sh_state, String ID){
  Serial.print(ID); Serial.print("_firing:"); Serial.println(sh_state->firing);
}

void DriverInput_serial_event(DriverInput*){

}

void dump_DriverInput(DriverInput* di){
  Serial.print("/DL");
  Serial.print(di->leftStickX); Serial.print(","); 
  Serial.println(di->leftStickY);
  
  Serial.print("/DR");
  Serial.print(di->rightStickX); Serial.print(","); 
  Serial.println(di->rightStickY);
  
  Serial.print("/DS"); 
  Serial.print(di->leftSwitch); Serial.print(","); 
  Serial.println(di->rightSwitch);
  
  Serial.print("/DM");
  Serial.print(di->mouseX); Serial.print(","); 
  Serial.println(di->mouseY);
}

void dump_RefSystem_State(S_RefSystem* rf){

}

void serial_event(C_Robot* config, S_Robot* state){
  char cmd = Serial.read();
  switch (cmd)
  {
    case 'G':
      Gimbal_serial_event(&config->gimbal, &state->gimbal);
      break;

    case 'V':
      Shooter17_serial_event(&config->shooter17, &state->shooter17);
      break;

    case 'F':
      Shooter42_serial_event(&config->shooter42, &state->shooter42);
      break;

    case 'R':
      RailChassis_serial_event(&config->railChassis, &state->chassis);
      break;

    case 'S':
      SwerveChassis_serial_event(&config->swerveChassis, &state->chassis);
      break;
  }
}

void dump_Robot(C_Robot* r_config, S_Robot* r_state){
  // dump_Chassis(&r_state->chassis);
  // dump_RailChassis(&r_config->railChassis);
  // dump_SwerveChassis(&r_config->swerveChassis);

  dump_Gimbal(&r_state->gimbal);
  dump_Gimbal(&r_config->gimbal);

  // dump_RefSystem_State(&r_state->refSystem);

  // dump_DriverInput(r_state->driverInput);
}


