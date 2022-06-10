#include <bitset>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "Send.h"
#include "DrawMessage.cpp"
#include "RefSend.cpp"
#include "Structures.cpp"
#include "RefSysGenUse.cpp"

using namespace std;

/*Draw::Draw(){
    this->setHeader(nullptr);
    this->setContent(nullptr);
    this->setFooter(nullptr);
}
Draw::Draw(Header* header, Content* content, Footer* footer){
    this->setHeader(header);
    this->setContent(content);
    this->setFooter(footer);
}*/
void Draw::drawWithColor() throw (exception){
    if (getColorID() == -1){
        throw exception();
    }
    this->count++;
};
void Draw::setColor(string color){
    this->color = color;
};
string Draw::getColor(){
    return this->color;
};
int Draw::getColorID(){
    int num = -1;
    const string COLORS[] = {"blue", "yellow", "green", "orange", "purplish red", "pink", "cyan", "black", "white"};
    if (this->color == "red"){
        return 0x0;
    }
    else{
        for (int color_index = 0; color_index < sizeof(COLORS)/sizeof(COLORS[0]); color_index++){
            if (COLORS[color_index] == this->color){
                num = color_index;
            }
        }
        return num;
    }
};
vector<uint8_t> Draw::lineData(tuple<int, int> v1, tuple<int, int> v2){
    drawWithColor();
    const string count = to_binaryString(this->count, 24);
    const string gc = string("001") + string("000") + string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const string gc2 = string("0000001010") + to_binaryString(get<0>(v1), 11) + to_binaryString(get<1>(v1), 11);
    const string gc3 = string("0001100100") + to_binaryString(get<0>(v2), 11) + to_binaryString(get<1>(v2), 11);
    bitset<120> bits(count + gc + gc2 + gc3);
    string result = bits.to_string();
    return to_bytes(result);
};
vector<uint8_t> Draw::rectData(int width, int height, tuple<int, int> center){
    drawWithColor();
    const int startX = get<0>(center) - width/2;
    const int startY = get<1>(center) + height/2;
    const int endX = get<0>(center) + width/2;
    const int endY = get<1>(center) - height/2;
    return rectData(tuple<int, int>(startX, startY), tuple<int, int>(endX, endY));
    
};
vector<uint8_t> Draw::rectData(tuple<int, int> topLeft, tuple<int, int> bottomRight){
    drawWithColor();
    const int startX = get<0>(topLeft);
    const int startY = get<1>(topLeft);
    const int endX = get<0>(bottomRight);
    const int endY = get<1>(bottomRight);
    const string count = to_binaryString(this->count, 24);
    const string gc = string("001") + string("001") + string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const string gc2 = string("0000001010") + to_binaryString(startX, 11) + to_binaryString(startY, 11);
    const string gc3 = string("0001100100") + to_binaryString(endX, 11) + to_binaryString(endY, 11);
    bitset<120> bits(count + gc + gc2 + gc3);
    //0x000001, 0b001, 0b001, 0b0001, 0b0111, 0b(18 0s), 0b000001010, startx, starty, 0b01100100, endx, endy
    string result = bits.to_string();
    //cout << result << endl;
    return to_bytes(result);
};
vector<uint8_t> Draw::circleData(tuple<int, int> center, int radius){
    drawWithColor();
    const string count = to_binaryString(this->count, 24);
    const string gc = string("001") + string("010") + string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const string gc2 = string("0000001010") + to_binaryString(get<0>(center), 11) + to_binaryString(get<1>(center), 11);
    const string gc3 = to_binaryString(radius, 11) + "0000000000000000000000";
    bitset<120> bits(count + gc + gc2 + gc3);
    string result = bits.to_string();
    //cout << result << endl;
    return to_bytes(result);
};
vector<uint8_t> Draw::textData(string text, int fontSize, tuple<int, int> topLeft){
    drawWithColor();
    const string count = to_binaryString(this->count, 24);
    const string gc = string("001") + string("111") + string("0001") + to_binaryString(getColorID(),4) + to_binaryString(fontSize, 9) + to_binaryString(text.length(), 9);
    const string gc2 = to_binaryString(fontSize/10, 10) + to_binaryString(get<0>(topLeft), 11) + to_binaryString(get<1>(topLeft), 11);
    const string gc3 = to_binaryString(fontSize, 10) + "0000000000000000000000";
    bitset<120> bits(count + gc + gc2 + gc3);
    string result = bits.to_string();
    //cout << result << endl;
    return to_bytes(result);
};
vector<uint8_t> Draw::createMessage(){
    vector<uint8_t> bytes = vector<uint8_t>();
    if ((this->getHeader() != nullptr) && (this->getContent() != nullptr) && (this->getFooter() != nullptr)){
        if ((this->getContent()->cmd_id = 0x0301) && (this->getContent()->message != nullptr)){
            bytes.push_back(this->getHeader()->SOF);
            bytes.push_back(this->getHeader()->data_length >> 8);
            bytes.push_back(this->getHeader()->data_length & 0xff);
            bytes.push_back(this->getHeader()->seq);
            bytes.push_back(this->getHeader()->CRC8);
            bytes.push_back(this->getContent()->cmd_id >> 8);
            bytes.push_back(this->getContent()->cmd_id & 0xff);
            vector<uint8_t> data = getContent()->message->getData();
            bytes.insert(bytes.end(), data.begin(), data.end());
            bytes.push_back(this->getFooter()->frame_tail >> 8);
            bytes.push_back(this->getFooter()->frame_tail & 0xff);
        }
    }
    else{
        cout << "error: no header or content or footer" << endl;
    }
    return bytes;
};

/*int main(){
    //const DATA_LENGTH, CRC8, FRAME_TAIL;
    const unsigned char SEQ = 0x0;
    const unsigned short CMD_ID = 0x0301;
    const unsigned short SENDER_ID = 0x1234;
    const unsigned short RECEIVER_ID = 0x1234;
    //1. Create draw
    Draw draw;
    //2. TODO: Create drawing & data
    vector<uint8_t> shape = draw.rectData(20,10,tuple<int,int>(5,5));
    DrawMessage drawMsg = DrawMessage(SENDER_ID, RECEIVER_ID, shape);
    //3. Create content
    Content content = Content(CMD_ID, &drawMsg);
    draw.setContent(&content);
    //4. Create header
    Header header = Header(&content, SEQ);
    draw.setHeader(&header);
    //5. Create footer
    Footer footer = Footer(&content);
    draw.setFooter(&footer);
    //6. Create message
    vector<uint8_t> msg = draw.createMessage();
    //TESTS:
    cout << "Test: rect" << endl;
    for (uint8_t c:shape){
        cout << "0x";
        cout << hex << int(c);
        cout << " ";
    }
    cout << endl << "Test: msg" << endl;
    for (uint8_t c:msg){
        cout << "0x";
        cout << hex << int(c);
        cout << " ";
    }
}*/
