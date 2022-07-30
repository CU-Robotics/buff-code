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
    int16_t CH0;
    int16_t CH1;
    int16_t CH2;
    int16_t CH3;

    //switches
    byte s1;
    byte s2;

    //keys
    //byte 1
    bool w;
    bool a;
    bool s;
    bool d;
    bool q;
    bool e;
    bool shift;
    bool ctrl;
    //byte 2
    bool r;
    bool f;
    bool g;
    bool z;
    bool x;
    bool c;
    bool v;
    bool b;

    //mouse
    int16_t mouseX;
    int16_t mouseY;
    int16_t mouseZ;
    bool mouseLeft;
    bool mouseRight;

    //Wheel on reciever
    int16_t remoteWheel;
};