#include "state/state.h"
#include "state/config.h"

void PID_serial_event(C_PID*);
void PID_serial_event(S_PID*);
void dump_PID_Config(C_PID*, String);
void dump_PID_State(S_PID*, String);


void SwerveModule_serial_event(C_SwerveModule*);
void SwerveModule_serial_event(S_SwerveModule*);
void dump_Swerve(C_SwerveModule*, String);
void dump_Swerve(S_SwerveModule*, String);


void RailChassis_serial_event(C_RailChassis*);
void SwerveChassis_serial_event(C_SwerveChassis*);
void Chassis_serial_event(S_Chassis*);
void dump_RailChassis(C_RailChassis*, String);
void dump_SwerveChassis(C_SwerveChassis*, String);
void dump_Chassis(S_Chassis*, String);

void Gimbal_serial_event(C_Gimbal*);
void Gimbal_serial_event(S_Gimbal*);
void dump_Gimbal(C_Gimbal*, String);
void dump_Gimbal(S_Gimbal*, String);


void Shooter17_serial_event(C_Shooter17*);
void Shooter42_serial_event(C_Shooter42*);
void Shooter_serial_event(S_Shooter*);
void dump_Shooter_State(S_Shooter*, String);


void DriverInput_serial_event(DriverInput*);
void dump_DriverInput(DriverInput*, String);

void dump_RefSystem_State(S_RefSystem*, String);

void serial_event(S_Robot*, C_Robot*);
void dump_Robot(S_Robot*, C_Robot*);
