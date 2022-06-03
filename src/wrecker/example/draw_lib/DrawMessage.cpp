#include <vector>
#include "Send.h"


/*DrawMessage::DrawMessage(){
    this->setSenderID(0);
    this->setReceiverID(0);
    this->setDataSegment(vector<uint8_t>());
}*/
DrawMessage::DrawMessage(unsigned short sender_id, unsigned short receiver_id, vector<uint8_t> data_segment){
    this->setSenderID(sender_id);
    this->setReceiverID(receiver_id);
    this->setDataSegment(data_segment);
}

void DrawMessage::setSenderID(unsigned short sender_id){
    this->sender_id = sender_id;
}

void DrawMessage::setReceiverID(unsigned short receiver_id){
    this->receiver_id = receiver_id;
}

void DrawMessage::setDataSegment(vector<uint8_t> data_segment){
    this->data_segment = data_segment;
}

unsigned short DrawMessage::getDataID(){
    return this->data_id;
}

unsigned short DrawMessage::getSenderID(){
    return this->sender_id;
}

unsigned short DrawMessage::getReceiverID(){
    return this->receiver_id;
}

vector<uint8_t> DrawMessage::getDataSegment(){
    return this->data_segment;
}

vector<uint8_t> DrawMessage::getData(){
    vector<uint8_t> bytes = vector<uint8_t>();
    bytes.push_back(data_id >> 8);
    bytes.push_back(data_id & 0xff);
    bytes.push_back(sender_id >> 8);
    bytes.push_back(sender_id & 0xff);
    bytes.push_back(receiver_id >> 8);
    bytes.push_back(receiver_id & 0xff);
    vector<uint8_t> dataSegment = this->getDataSegment();
    bytes.insert(bytes.end(), dataSegment.begin(), dataSegment.end());
    return bytes;
}