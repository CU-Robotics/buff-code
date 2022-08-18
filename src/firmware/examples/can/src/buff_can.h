#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef BUFFCAN_H
#define BUFFCAN_H

class BuffCan {
	private:
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

        CAN_message_t input[2][3];

        unsigned long d_t;

    public:
        int id = -1;
        BuffCan();
        void set(int, int, int, byte, byte);
        void write();
        void read();
};

#endif