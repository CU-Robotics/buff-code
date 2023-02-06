#include "sensors/dr16.h"

enum Mode {
  IDLE,
  MANUAL,
  AUTO
};

struct GlobalRobotState {
  DR16 receiver;

  Mode chassisMode = Mode::IDLE;
  float chassisHeading; // degrees relative to global north
  float chassisHeadingRate; // degrees/sec

  Mode gimbalMode = Mode::IDLE;
  float gimbalHeading[2]; // (yaw, pitch): degrees relative to global north
  float gimbalHeadingRate[2]; // (yawRate, pitchRate): degrees/sec

  Mode shooterMode = Mode::IDLE;
};