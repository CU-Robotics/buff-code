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


    //controller input
    //Sticks
    int CH0;
    int CH1;
    int CH2;
    int CH3;

    //switches
    byte S1;
    byte S2;

    //keys
    bool w;
    bool a;
    bool s;
    bool d;
    bool f;
};