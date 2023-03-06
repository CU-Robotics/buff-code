#include "dr16.h"

int16_t normalize_channel(int16_t value){
    return int16_t((float(value) - 1024.0) / 700.0 * pow(2, 15));
}

void print_receiver_input(byte* buffer){
    for (int i = 0; i < 18; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            Serial.print(bitRead(buffer[i], j));
        }

        Serial.print(" ");
        if ((i + 1) % 6 == 0) {
            Serial.println();
        }
    }
    Serial.println("\n===========");
}

DR16::DR16()
{
    d_t = micros();
    Serial5.clear();
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
}

void DR16::read(byte *buffer, int offset)
{

    if (Serial5.available() < 18)
    {
        return;
    }
    else if (Serial5.available() % 18 != 0)
    {
        Serial5.clear();
        return;
    }

    if (Serial5.available() >= 18)
    {
        byte tmp[18];
        Serial5.readBytes(tmp, 18);
        d_t = micros();

        buffer[offset] = 2;

        // debugging
        // print_receiver_input(tmp);

        /*
         DR16 data format
            0:  [ch0.7,   ch0.6,   ch0.5,   ch0.4,   ch0.3,   ch0.2,   ch0.1,   ch0.0]
            1:  [ch1.4,   ch1.3,   ch1.2,   ch1.1,   ch1.0,  ch0.10,   ch0.9,   ch0.8]
            2:  [ch2.1,   ch2.0,  ch1.10,   ch1.9,   ch1.8,   ch1.7,   ch1.6,   ch1.5]
            3:  [ch2.9,   ch2.8,   ch2.7,   ch2.6,   ch2.5,   ch2.4,   ch2.3,   ch2.2]
            4:  [ch3.6,   ch3.5,   ch3.4,   ch3.3,   ch3.2,   ch3.1,   ch3.0,  ch2.10]
            5:  [s2H,       s2L,     s1H,     s1L,  ch3.10,   ch3.9,   ch3.8,   ch3.7]
            6:  [                              mouse_x_L                             ]
            7:  [                              mouse_x_H                             ]
            8:  [                              mouse_y_L                             ]
            9:  [                              mouse_y_H                             ]
            10: [                              mouse_z_L                             ]
            11: [                              mouse_z_H                             ]
            12: [lmb,       lmb,     lmb,     lmb,     lmb,     lmb,     lmb,     lmb]
            13: [rmb,       rmb,     rmb,     rmb,     rmb,     rmb,     rmb,     rmb]
            14: [key,       key,     key,     key,     key,     key,     key,     key]
            15: [key,       key,     key,     key,     key,     key,     key,     key]
            16: [                              reserved                              ]
            17: [                                text                                ]

         HID packet format
            0:  [                                 0x02                               ]
            1:  [0,           0,       0,      0,      s2H,     s2L,     s1H,     s1L] (switches)
            2:  [                                 lmb                                ] (mouse buttons)
            3:  [                                 rmb                                ]
            4:  [                            ch0_normal_L                            ] (joysticks)
            5:  [                            ch0_normal_H                            ]
            6:  [                            ch1_normal_L                            ]
            7:  [                            ch1_normal_H                            ]
            8:  [                            ch2_normal_L                            ]
            9:  [                            ch2_normal_H                            ]
            10: [                            ch3_normal_L                            ]
            11: [                            ch3_normal_H                            ]
            12: [                              mouse_x_L                             ] (mouse)
            13: [                              mouse_x_H                             ]
            14: [                              mouse_y_L                             ]
            15: [                              mouse_y_H                             ]
            16: [                              mouse_z_L                             ]
            17: [                              mouse_z_H                             ]
            18: [                             keyboard_L                             ] (keyboard buttons)
            19: [                             keyboard_H                             ]
        */

        int16_t r_stick_x = normalize_channel(((tmp[1] & 0b00000111) << 8) | tmp[0]);
        int16_t r_stick_y = normalize_channel(((tmp[2] & 0b11111100) << 5) | ((tmp[1] & 0b11111000) >> 3));
        int16_t l_stick_x = normalize_channel((((tmp[4] & 0b00000001) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0b11000000) >> 6));
        int16_t l_stick_y = normalize_channel(((tmp[5] & 0b00001111) << 7) | ((tmp[4] & 0b11111110) >> 1));

        // Serial.println("norm vals");
        // Serial.print(r_stick_x); Serial.print(" "); Serial.println(((tmp[1] & 0b00000111) << 8) | tmp[0]);
        // Serial.print(r_stick_y); Serial.print(" "); Serial.println(((tmp[2] & 0b11111100) << 5) | ((tmp[1] & 0b11111000) >> 3));
        // Serial.print(l_stick_x); Serial.print(" "); Serial.println((((tmp[4] & 0b00000001) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0b11000000) >> 6));
        // Serial.print(l_stick_y); Serial.print(" "); Serial.println(((tmp[5] & 0b00001111) << 7) | ((tmp[4] & 0b11111110) >> 1));


        buffer[offset+1] = ((tmp[5] & 0b11110000) >> 4);   // switches
        buffer[offset+2] = tmp[12];                        // lmb
        buffer[offset+3] = tmp[13];                        // rmb
        buffer[offset+4] = (r_stick_x >> 8) & 0xff;       // ch 0 high
        buffer[offset+5] = r_stick_x && 0xff;              // ch 0 low
        buffer[offset+6] = (r_stick_y >> 8) & 0xff;       // ch 1 high
        buffer[offset+7] = r_stick_y && 0xff;              // ch 1 low
        buffer[offset+8] = (l_stick_x >> 8) & 0xff;       // ch 2 high
        buffer[offset+9] = l_stick_x && 0xff;              // ch 2 low
        buffer[offset+10] = (l_stick_y >> 8) & 0xff;      // ch 3 high
        buffer[offset+11] = l_stick_y && 0xff;             // ch 3 low
        buffer[offset+12] = tmp[7];                        // mouse_x high
        buffer[offset+13] = tmp[6];                        // mouse_x low
        buffer[offset+14] = tmp[9];                        // mouse_y high
        buffer[offset+15] = tmp[8];                        // mouse_y low
        buffer[offset+16] = tmp[11];                       // mouse_z high
        buffer[offset+17] = tmp[10];                       // mouse_z low
        buffer[offset+18] = tmp[15];                       // keyboard high
        buffer[offset+19] = tmp[14];                       // keyboard low
        
        // print_receiver_input(&buffer[offset]);
    }
}