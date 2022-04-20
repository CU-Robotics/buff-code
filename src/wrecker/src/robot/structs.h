#define STRUCTS_H

#ifndef _FLEXCAN_T4_H_
#include <FlexCAN_T4.h>
#endif

struct RobotConfig {
    short pitchID;
    short yawID;

    float pitchP;
    float pitchI;
    float pitchD;

    float yawP;
    float yawI;
    float yawD;
};

struct RobotInput {
    float gimbalPitch;
    float gimbalYaw;
};