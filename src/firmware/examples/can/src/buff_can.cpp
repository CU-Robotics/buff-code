#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "buff_can.h"

int16_t bytes_to_int16_t(byte upper, byte lower) {
    /*
          Helper to convert bytes to int16_t.
          turn this into a macro some day.
        @param
            upper: upper byte of value
            lower: lower byte of value
        @return
            value
    */
    return int16_t(upper << 8) | lower;
}

int16_t ang_from_can_bytes(byte b1, byte b2){
    /*
          Getter for the angle in can bytes.
        @param
            b1: the upper byte
            b2: the lower byte 
        @return
            angle: the angle scaled to 0-36000
    */
    return map(bytes_to_int16_t(b1, b2), 0, 8191, 0, 32000);
}

void print_rm_feedback_struct(RM_Feedback_Packet* fb) {
    /*
          print function for a feedback packet
        @param
            fb: feedback struct
        @return
            None
    */
    Serial.print("\t[");
    Serial.print(fb->angle);
    Serial.print("\t");
    Serial.print(fb->RPM);

    Serial.print("\t");
    Serial.print(fb->torque);
    Serial.print("\t");
    Serial.print(ARM_DWT_CYCCNT - fb->timestamp);  
    Serial.println("]");
}

void print_rm_config_struct(RM_CAN_Device* dev) {
    /*
          print function for a feedback packet
        @param
            dev: CAN config struct
        @return
            None
    */
    Serial.println("\t======");
    Serial.print("\t[");
    Serial.print(dev->can_bus);
    Serial.print("\t");
    Serial.print(dev->message_type);
    Serial.print("\t");
    Serial.print(dev->message_offset);
    Serial.print("\t");
    Serial.print(dev->motor_id);
    Serial.print("\t");
    Serial.print(dev->esc_id);
    Serial.print("\t");
    Serial.print(dev->return_id, HEX);
    Serial.print("\t");
    Serial.print(dev->motor_type);
    Serial.println("]");
    print_rm_feedback_struct(dev->feedback);
}

void print_can_message(CAN_message_t* msg) {
    /*
          main serial print the data in a can message struct.
        @param
            msg: can message struct
        @return
            None
    */
    Serial.println("\t======");
    Serial.print("\tid: "); Serial.println(msg->id, HEX);
    Serial.print("\t\t[");
    Serial.print(msg->buf[0]);
    Serial.print("\t");
    Serial.print(msg->buf[1]);

    Serial.print("\t");
    Serial.print(msg->buf[2]);
    Serial.print("\t");
    Serial.print(msg->buf[3]);
    Serial.print("\t");
    Serial.print(msg->buf[4]);
    Serial.print("\t");
    Serial.print(msg->buf[5]);
    Serial.print("\t");
    Serial.print(msg->buf[6]);
    Serial.print("\t");
    Serial.print(msg->buf[7]);  
    Serial.println("]");
}

void prettyprint_can_message(CAN_message_t* msg) {
    /*
          Pretty version of the can message print.
        for return messages.
        @param
            msg: can message to print
        @return
            None
    */
    Serial.println("\t======");
    Serial.print("\tid: "); Serial.println(msg->id - 0x200, HEX);
    Serial.print("\t\t");
    Serial.print(map((uint16_t(msg->buf[0]) << 8) | uint16_t(msg->buf[1]), 0, 8191, 0, 360));
    Serial.print("\t");
    Serial.print((int16_t(msg->buf[2]) << 8) | int16_t(msg->buf[3]));
    Serial.print("\t");
    Serial.println((int16_t(msg->buf[4]) << 8) | int16_t(msg->buf[5]));
}

BuffCan::BuffCan(){
    /*
          BuffCan constructor, initializes can busses and sets can messages and indices to defaults.
        @param
            None
        @return
            BuffCan: Can manager code
    */
    // start can1 and can2
    can1.begin();
    can1.setBaudRate(1000000);
    can2.begin();
    can2.setBaudRate(1000000);

    // set each message id for both can busses, not sure if it needs to be reset sometimes or what
    for (int i = 0; i < 2; i++){
        output[i][0].id = 0x200;
        output[i][1].id = 0x1FF;
        output[i][2].id = 0x2FF;
    }
    for (int j = 0; j < MAX_CAN_RETURN_IDS; j++) {
        can1_motor_index[j] = -1;
        can2_motor_index[j] = -1;
    }
}

// struct RM_CAN_Device {
//     int8_t can_bus;        // bus the device is connected to (1, 2)
//     int8_t message_type;   // 0: 0x200, 1: 0x1FF, 2: 0x2FF
//     int8_t message_offset; // 0-6 (always even)

//     int8_t esc_id;         // 1-8 the blinking light
//     int8_t motor_id;       // index of motor in the serialized motor structure
//     int8_t return_id;      // id of can message returned from device (- 0x201 to store as int8_t)

//     int8_t motor_type;     // 0: C6XX, 1: GM6020
//     RM_Feedback_Packet* feedback;
// };
void BuffCan::set_index(byte index[MAX_NUM_RM_MOTORS][3]){
    /*
          Initialize the index for object searches.
        @param
            index: 16 motors with three values each to define 
                the can packet mapping.
        @return
            None
    */
    for (int8_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {

        int8_t cb = index[i][0] - 1;
        int8_t mt = index[i][1];
        int8_t eid = index[i][2];

        int8_t mgt = int(index[i][2] / 4) + index[i][1];          // message type = (esc ID / 4) + motor_type 
        int8_t mo = (2 * ((index[i][2] - 1) % 4));                // message offset = (esc ID % 4) - 1
        int8_t rid = index[i][2] + (5 * index[i][1]) - 1;         // 0x200 + esc ID + motor_type_offset = rid (store as 8 bit though - 0x201)

        RM_Feedback_Packet* fb = new RM_Feedback_Packet{0, 0, 0, ARM_DWT_CYCCNT};
        motor_index[i] = new RM_CAN_Device {cb, mgt, mo, eid, i, rid, mt, fb};

        if (cb >= 0) {
            switch(cb) {
                case 0:
                    can1_motor_index[rid] = i;
                    break;

                case 1:
                    can2_motor_index[rid] = i;
                    break;

                default:
                    break;
            }

            num_motors += 1;
        }
    }
}

int8_t BuffCan::esc_id_from_motor_idx(int idx){
    /*
          Getter for the esc ID number.
        @param
            idx: index of the motor
        @return
            esc_id
    */
    return motor_index[idx]->esc_id;
}

int8_t BuffCan::can_bus_from_motor_idx(int idx){
    /*
          Getter for the can bus number.
        @param
            idx: index of the motor
        @return
            can_bus: 1, 2... maybe 3
    */
    return motor_index[idx]->can_bus;
}

int8_t BuffCan::message_type_from_motor_idx(int idx){
    /*
          Getter for the message type.
        @param
            idx: index of the motor
        @return
            message_type: 0: 0x200, 1: 0x1FF, 2: 0x2FF
    */
    return motor_index[idx]->message_type;
}

int8_t BuffCan::message_offset_from_motor_idx(int idx){
    /*
          Getter for the message offset.
        @param
            idx: index of the motor
        @return
            message_offset: 0 - 3
    */
    return motor_index[idx]->message_offset;
}

int8_t BuffCan::motor_idx_from_return(int can_bus, int return_id){
    /*
          Getter for the motor index.
        @param
            can_bus: can bus number (number not index)
            return_id: id of can message replied from motors
        @return
            idx: 0-MAX_NUM_MOTORS
    */
    if (return_id - 0x201 >= 0 && return_id - 0x201 < MAX_CAN_RETURN_IDS) {
        switch (can_bus) {
            case 1: 
                return can1_motor_index[return_id - 0x201];
            case 2:
                return can2_motor_index[return_id - 0x201];

            default:
                return -1;
        }
    }    
    return -1;
}

void BuffCan::zero_can() {
    /*
          Zero out the values in the can packets,
        this shuts down the motors.
        @param
            None
        @return
            None
    */
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 8; j++) {
            output[0][i].buf[j] = 0;
            output[1][i].buf[j] = 0;
        }
    }
}

void BuffCan::write_can(){
    /*
          Write the can output packets to the can busses.
        @param
            None
        @return
            None
    */
    for (int i = 0; i < 3; i++){
        can1.write(output[0][i]);
        can2.write(output[1][i]);
    }
}

void BuffCan::set_output(int16_t values[MAX_NUM_RM_MOTORS]){
    /*
          Send the motor command from serial to the
        can packets.
        @param
            values: values to set in the can packet
        @return
            None
    */

    int can_bus;
    int msg_type;
    int msg_offset;

    for (int i = 0; i < num_motors; i++) {
        can_bus = motor_index[i]->can_bus;
        msg_type = motor_index[i]->message_type;
        msg_offset = motor_index[i]->message_offset;

        // The can busses are numbered 1-2 (indexed 0-1)
        output[can_bus][msg_type].buf[msg_offset] = highByte(values[i]);
        output[can_bus][msg_type].buf[msg_offset + 1] = lowByte(values[i]);
    }
}



void BuffCan::set_feedback(int can_bus, CAN_message_t* msg){
    /*
          Set the motors feedback into the motor index.
        @param
            can_bus: number of can bus
            msg: CAN message object
        @return
            None
    */
    int ret_id = msg->id;

    if (ret_id > 0){
        int motor_id = motor_idx_from_return(can_bus, ret_id);

        if (motor_id >= 0) {
            motor_index[motor_id]->feedback->angle = ang_from_can_bytes(msg->buf[0], msg->buf[1]);
            motor_index[motor_id]->feedback->RPM = bytes_to_int16_t(msg->buf[2], msg->buf[3]);
            motor_index[motor_id]->feedback->torque = bytes_to_int16_t(msg->buf[4], msg->buf[5]);
            motor_index[motor_id]->feedback->timestamp = ARM_DWT_CYCCNT;
        }
    }
}

void BuffCan::read_can1(){
    /*
          read can bus 1
        @param
            buff: buffer to save data too
        @return
            None
    */
    CAN_message_t tmp;
    can1.read(tmp);
    set_feedback(1, &tmp);
}

void BuffCan::read_can2(){
    /*
          read can bus 2
        @param
            buff: buffer to save data too
        @return
            None
    */
    CAN_message_t tmp;
    can2.read(tmp);
    set_feedback(2, &tmp);
}
