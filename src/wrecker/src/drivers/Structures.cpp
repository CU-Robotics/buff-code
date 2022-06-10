//#pragma once

#include <vector>
#include "Send.h"
#include "Checksum.cpp"

using namespace std;

Content::Content(unsigned short cmd_id, Data* message){
    this->cmd_id = cmd_id;
    this->message = message;
}

Header::Header(Content* content, unsigned char seq){
    const unsigned short length = content->message->getData().size() + 2;
    this->data_length = length;
    this->seq = seq;
    unsigned char temp[] = {uint8_t(length >> 8), uint8_t(length & 0xff), seq};
    this->CRC8 = Get_CRC8_Check_Sum(temp, 3, 0x00);
}

Footer::Footer(Content* content){
    vector<uint8_t> temp = content->message->getData();
    temp.insert(temp.begin(), uint8_t(content->cmd_id & 0xff));
    temp.insert(temp.begin(), uint8_t(content->cmd_id >> 8));
    this->frame_tail = Get_CRC16_Check_Sum(&temp[0], temp.size(), 0x00);
}
