#include "state/state.h"
#include "state/config.h"

void PID_serial_event(C_PID*, S_PID*);
void dump_PID(C_PID*, String);
void dump_PID(S_PID*, String);

void detection_serial_event(S_Gimbal*);

void SwerveModule_serial_event(C_SwerveModule*, S_SwerveModule*);
void dump_Swerve(C_SwerveModule*, String);
void dump_Swerve(S_SwerveModule*, String);

void RailChassis_serial_event(C_RailChassis*, S_Chassis*);
void SwerveChassis_serial_event(C_SwerveChassis*, S_Chassis*);
void dump_RailChassis(C_RailChassis*, String);
void dump_SwerveChassis(C_SwerveChassis*, String);
void dump_Chassis(S_Chassis*, String);

void Gimbal_serial_event(C_Gimbal*, S_Gimbal*);
void dump_Gimbal(C_Gimbal*);
void dump_Gimbal(S_Gimbal*);


void Shooter17_serial_event(C_Shooter17*, S_Shooter*);
void Shooter42_serial_event(C_Shooter42*, S_Shooter*);
void dump_Shooter_State(S_Shooter*, String);


void DriverInput_serial_event(DriverInput*);
void dump_DriverInput(DriverInput*);

void dump_RefSystem_State(S_RefSystem*);

void serial_event(C_Robot*, S_Robot*);
void dump_Robot(C_Robot*, S_Robot*);
