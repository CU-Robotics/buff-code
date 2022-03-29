// TODO: Implement RobotInput

class RobotInput {
  public:
    RobotInput();
    void update();

    // DR16 Input
    bool[4] keyWASD();
    bool keyQ();
    bool keyE();
    bool keyR();
    bool keyShift();
    bool keyCtrl();
    bool keySpace();
    bool keyMouse1();
    bool keyMouse2();
    float mouseX();
    float mouseY();

    // Global Sensors
    float getChassisGlobalHeading(); // Degrees, obtained from IMU. Zeroed on startup.
    float getChassisRelativeHeading(); // Degrees, obtained from IMU & GM6020-Yaw's Encoder.

    // Ref System
    int getRobotLevel();
    int getRobotType();
    int getMatchState();
    float getMatchTime();
    // Add more
};