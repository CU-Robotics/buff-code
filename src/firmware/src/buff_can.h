#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef BUFFCAN_H
#define BUFFCAN_H

class BuffCan {
	private:
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

        uint16_t output[24];
        unsigned long d_t;

    public:
        CAN_message_t input[2][3];

        int id = -1;
        BuffCan();
        void zero_can();
        void write();
        void set_input(int, int, byte*);
        int set_output(int, CAN_message_t*, byte*);
        void read_can1(byte*);
        void read_can2(byte*);
};

#endif