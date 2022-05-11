

void PID_serial_event(C_PID);
void PID_serial_event(S_PID);
void dump_PID_Config(C_PID, String);
void dump_PID_State(S_PID, String);


void SwerveModule_serial_event(C_SwerveModule);
void SwerveModule_serial_event(S_SwerveModule);
void dump_Swerve_Config(C_SwerveModule, String);
void dump_Swerve_State(S_SwerveModule, String);


void RailChassis_serial_event(C_RailChassis);
void SwerveChassis_serial_event(C_SwerveChassis);
void Chassis_serial_event(S_Chassis);
void dump_RailChassis_Config(C_RailChassis, String);
void dump_SwerveChassis_Config(C_SwerveChassis, String);
void dump_Chassis_State(S_Chassis, String);

void Gimbal_serial_event(C_Gimbal);
void Gimbal_serial_event(S_Gimbal);
void dump_Gimbal_Config(C_Gimbal, String);
void dump_Gimbal_State(S_Gimbal, String);

void Shooter17_serial_event(C_Shooter17);
void Shooter42_serial_event(C_Shooter42);
void Shooter_serial_event(S_Shooter);
void dump_Shooter_State(S_Shooter, String);

void DriverInput_serial_event(DriverInput);
void dump_DriverInput(DriverInput, String);

void dump_RefSystem_State(S_RefSystem, String);

void dump_Robot_State(S_Robot);
