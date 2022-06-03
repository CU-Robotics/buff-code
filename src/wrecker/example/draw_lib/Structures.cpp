#pragma once

#include "Send.h"

using namespace std;

Header::Header(unsigned short data_length, unsigned char seq, unsigned char CRC8){
    this->data_length = data_length;
    this->seq = seq;
    this->CRC8 = CRC8;
}

Content::Content(unsigned short cmd_id, Data* message){
    this->cmd_id = cmd_id;
    this->message = message;
}

Footer::Footer(unsigned short frame_tail){
    this->frame_tail = frame_tail;
}