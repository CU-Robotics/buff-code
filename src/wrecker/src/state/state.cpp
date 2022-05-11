#include <Arduino.h>

#include "state/state.h"


void dump_PID_State(S_PID* pid, char* ID)
{
  Serial.print(ID); Serial.print("/Xp:"); Serial.print(pid->X[0]); Serial.print(",");
  Serial.print(ID); Serial.print("/Xi:"); Serial.print(pid->X[1]); Serial.print(",");
  Serial.print(ID); Serial.print("/Xd:"); Serial.print(pid->X[2]); Serial.print(",");
  Serial.print(ID); Serial.print("/Y:"); Serial.println(pid->Y);
}

void dump_SwerveChassis_State(S_SwerveChassis* sc_state, char* ID){
  Serial.print(ID); Serial.print("/heading:"); Serial.print(sc_state->heading); Serial.print(",");
  Serial.print(ID); Serial.print("/rpm:"); Serial.print(sc_state->rpm); Serial.print(",");
  Serial.print(ID); Serial.print("/alpha:"); Serial.print(sc_state->alpha); Serial.print(",");
  Serial.print(ID); Serial.print("/ax:"); Serial.println(sc_state->xAccel); Serial.print(",");
  Serial.print(ID); Serial.print("/ay:"); Serial.println(sc_state->yAccel);
}

void dump_RailChassis_State(S_RailChassis* rc_state, char* ID){
  Serial.print(ID); Serial.print("/p:"); Serial.print(rc_state->pos); Serial.print(",");
  Serial.print(ID); Serial.print("/v:"); Serial.print(rc_state->vel); Serial.print(",");
  Serial.print(ID); Serial.print("/a:"); Serial.println(rc_state->accel);
}

void dump_Gimbal_State(S_Gimbal* g_state, char* ID){
  Serial.print(ID); Serial.print("/pitch:"); Serial.print(g_state->pitch); Serial.print(",");
  Serial.print(ID); Serial.print("/yaw:"); Serial.println(g_state->yaw);
  Serial.print(ID); Serial.print("/yawGlobal:"); Serial.print(g_state->yawGlobal); Serial.print(",");
  dump_PID_State(&g_state->yaw_PID, "/swerve_chassis/yawPID");
  dump_PID_State(&g_state->pitch_PID, "/swerve_chassis/pitchPID");
}

void dump_Shooter_State(S_Shooter* sh_state, char* ID){
  Serial.print(ID); Serial.print("/firing:"); Serial.println(sh_state->firing);
}

void dump_DriverInput(DriverInput* di, char* ID){
  Serial.print(ID); Serial.print("/leftStickX:"); Serial.print(di->leftStickX); Serial.print(",");
  Serial.print(ID); Serial.print("/leftStickY:"); Serial.print(di->leftStickY); Serial.print(",");
  Serial.print(ID); Serial.print("/rightStickX:"); Serial.print(di->rightStickX); Serial.print(",");
  Serial.print(ID); Serial.print("/rightStickY:"); Serial.print(di->rightStickY); Serial.print(",");
  Serial.print(ID); Serial.print("/leftSwitch:"); Serial.print(di->leftSwitch); Serial.print(",");
  Serial.print(ID); Serial.print("/rightSwitch:"); Serial.print(di->rightSwitch); Serial.print(",");
  Serial.print(ID); Serial.print("/mouseX:"); Serial.print(di->mouseX); Serial.print(",");
  Serial.print(ID); Serial.print("/mouseY:"); Serial.println(di->mouseY);
}

void dump_Robot_State(S_Robot* r_state){
  dump_SwerveChassis_State(&r_state->swerve_chassis, "/swerve_chassis");
  dump_Gimbal_State(&r_state->gimbal, "/gimbal");
}


