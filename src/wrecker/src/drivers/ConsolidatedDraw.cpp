#include <bitset>
#include <iostream>
#include <math.h>
#include <string>
#include <tuple>
#include <vector>

using namespace std;

//GENERAL FUNCTIONS
/*
** Descriptions: Conversion of an integer to a binary string
** Input: integer, expected length of binary string
** Output: string (binary string)
*/
std::string to_binaryString(int num, int length){
    if (to_string(num).length() > length){
        return "error";
    }
    bitset<sizeof(num)> bits(num);
    std::string result = bits.to_string();
    while (result.length() < length){
        result = "0" + result;
    }
    return result;
}
/*
** Descriptions: Conversion of a binary string to a vector of bytes
** Input: string
** Output: vector<uint8_t> (vector of bytes)
*/
vector<uint8_t> to_bytes(std::string str){
    vector<uint8_t> bytes = vector<uint8_t>();
    if (str.length()%8 != 0){
        return bytes;
    }
    for (int _ = 0; _ < str.length()/8; _ ++){
        int num = 0;
        const std::string sub = str.substr(_*8, 8);
        for (int __ = 0; __ < 8; __++){
            num += (stoi(to_string(sub.at(__)))-48)*pow(2,7-__);
        }
        bytes.push_back(uint8_t(num));
    }
    return bytes;
}
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    } 
    return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/ 
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}
uint16_t CRC_INIT = 0xffff; 
const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}
/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return false;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

//CLASSES AND STRUCTS
class Data{
    public:
    virtual vector<uint8_t> getData() = 0;
};
class DrawMessage: public Data{
    private:
    //DRAW ONE GRAPHIC ONLY AT ONE TIME
    unsigned short data_id = 0x0101;
    unsigned short sender_id;
    unsigned short receiver_id;
    vector<uint8_t> data_segment;
    public:
    DrawMessage(unsigned short sender_id, unsigned short receiver_id, vector<uint8_t> data_segment);
    void setSenderID(unsigned short sender_id);
    void setReceiverID(unsigned short receiver_id);
    void setDataSegment(vector<uint8_t> data_segment);
    unsigned short getDataID();
    unsigned short getSenderID();
    unsigned short getReceiverID();
    vector<uint8_t> getDataSegment();
    vector<uint8_t> getData();
};
struct Content{
    unsigned short cmd_id;
    Data* message;
    Content(){cmd_id = 0; this->message = nullptr;};
    Content(unsigned short cmd_id, Data* message);
};
struct Header{
    unsigned char SOF = 0xA5;
    unsigned short data_length;
    unsigned char seq;
    unsigned char CRC8; 
    Header(){data_length = 0; seq = 0; CRC8 = 0;};
    Header(Content* content, unsigned char seq);
};
struct Footer{
    unsigned short frame_tail;
    Footer(){frame_tail = 0;};
    Footer(Content* content);
};
class RefSend{
    private:
    Header* header = nullptr;
    Content* content = nullptr;
    Footer* footer = nullptr;

    public:
    void setHeader(Header* header);
    void setContent(Content* content);
    void setFooter(Footer* footer);
    Header* getHeader();
    Content* getContent();
    Footer* getFooter();
    virtual vector<uint8_t> createMessage() = 0; //RETURN: array of bytes containing header, content, footer 
};
class Draw: public RefSend{
    private:
    //default color
    std::string color = "black";
    //index counter
    int count = 0;
    void drawWithColor() throw (exception); //uses private color to draw in color

    public:
    void setColor(std::string color);
    std::string getColor();
    int getColorID();
    vector<uint8_t> lineData(tuple<int, int> v1, tuple<int, int> v2); //RETURN: array of bytes to set data_segment
    vector<uint8_t> rectData(int width, int height, tuple<int, int> center); //RETURN: array of bytes to set data_segment
    vector<uint8_t> rectData(tuple<int, int> topLeft, tuple<int, int> bottomRight); //RETURN: array of bytes to set data_segment
    vector<uint8_t> circleData(tuple<int, int> center, int radius); //RETURN: array of bytes to set data_segment
    vector<uint8_t> textData(std::string text, int fontSize, tuple<int, int> topLeft); // RETURN: array of bytes to set data_segment
    vector<uint8_t> createMessage(); //RETURN: array of bytes containing header, content, footer
};
//CLASS FUNCTIONS
//Structs
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
//RefSend
void RefSend::setHeader(Header* header){
    this->header = header;
}
void RefSend::setContent(Content* content){
    this->content = content;
}
void RefSend::setFooter(Footer* footer){
    this->footer = footer;
}
Header* RefSend::getHeader(){
    return this->header;
}
Content* RefSend::getContent(){
    return this->content;
}
Footer* RefSend::getFooter(){
    return this->footer;
}
//DrawMessage
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
//Draw
void Draw::drawWithColor() throw (exception){
    if (getColorID() == -1){
        throw exception();
    }
    this->count++;
};
void Draw::setColor(std::string color){
    this->color = color;
};
std::string Draw::getColor(){
    return this->color;
};
int Draw::getColorID(){
    int num = -1;
    const std::string COLORS[] = {"blue", "yellow", "green", "orange", "purplish red", "pink", "cyan", "black", "white"};
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
    const std::string count = to_binaryString(this->count, 24);
    const std::string gc = std::string("001") + std::string("000") + std::string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const std::string gc2 = std::string("0000001010") + to_binaryString(get<0>(v1), 11) + to_binaryString(get<1>(v1), 11);
    const std::string gc3 = std::string("0001100100") + to_binaryString(get<0>(v2), 11) + to_binaryString(get<1>(v2), 11);
    bitset<120> bits(count + gc + gc2 + gc3);
    std::string result = bits.to_string();
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
    const std::string count = to_binaryString(this->count, 24);
    const std::string gc = std::string("001") + std::string("001") + std::string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const std::string gc2 = std::string("0000001010") + to_binaryString(startX, 11) + to_binaryString(startY, 11);
    const std::string gc3 = std::string("0001100100") + to_binaryString(endX, 11) + to_binaryString(endY, 11);
    bitset<120> bits(count + gc + gc2 + gc3);
    //0x000001, 0b001, 0b001, 0b0001, 0b0111, 0b(18 0s), 0b000001010, startx, starty, 0b01100100, endx, endy
    std::string result = bits.to_string();
    //cout << result << endl;
    return to_bytes(result);
};
vector<uint8_t> Draw::circleData(tuple<int, int> center, int radius){
    drawWithColor();
    const std::string count = to_binaryString(this->count, 24);
    const std::string gc = std::string("001") + std::string("010") + std::string("0001") + to_binaryString(getColorID(),4) + "000000000000000000";
    const std::string gc2 = std::string("0000001010") + to_binaryString(get<0>(center), 11) + to_binaryString(get<1>(center), 11);
    const std::string gc3 = to_binaryString(radius, 11) + "0000000000000000000000";
    bitset<120> bits(count + gc + gc2 + gc3);
    std::string result = bits.to_string();
    //cout << result << endl;
    return to_bytes(result);
};
vector<uint8_t> Draw::textData(std::string text, int fontSize, tuple<int, int> topLeft){
    drawWithColor();
    const std::string count = to_binaryString(this->count, 24);
    const std::string gc = std::string("001") + std::string("111") + std::string("0001") + to_binaryString(getColorID(),4) + to_binaryString(fontSize, 9) + to_binaryString(text.length(), 9);
    const std::string gc2 = to_binaryString(fontSize/10, 10) + to_binaryString(get<0>(topLeft), 11) + to_binaryString(get<1>(topLeft), 11);
    const std::string gc3 = to_binaryString(fontSize, 10) + "0000000000000000000000";
    bitset<120> bits(count + gc + gc2 + gc3);
    std::string result = bits.to_string();
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

//ABSTRACTED FUNCTIONS & CLASS
class AbstractDraw{
    unsigned short sender_id;
    unsigned short receiver_id;
    unsigned char seq = 0;
    std::string color = "black";
    AbstractDraw(std::string sender, std::string receiver, std::string team);
    void setColor(std::string color);
    vector<uint8_t> drawLine(tuple<int, int> v1, tuple<int, int> v2);
    vector<uint8_t> drawRect(int width, int height, tuple<int, int> center); //RETURN: array of bytes
    vector<uint8_t> drawRect(tuple<int, int> topLeft, tuple<int, int> bottomRight); //RETURN: array of bytes
    vector<uint8_t> drawCircle(tuple<int, int> center, int radius); //RETURN: array of bytes
    vector<uint8_t> drawText(std::string text, int fontSize, tuple<int, int> topLeft); // RETURN: array of bytes
};
AbstractDraw::AbstractDraw(std::string sender, std::string receiver, std::string team){
    if (team == "red"){
        if (sender == "infantry" || sender == "standard"){
            this->sender_id = 0x0103;
        }
        else if (sender == "hero"){
            this->sender_id = 0x0101;
        }
        if (receiver == "infantry" || receiver == "standard"){
            this->receiver_id = 3;
        }
        else if (receiver == "hero"){
            this->receiver_id = 1;
        }
    }
    else if (team == "blue"){
        if (sender == "infantry" || sender == "standard"){
            this->sender_id = 0x0167;
        }
        else if (sender == "hero"){
            this->sender_id = 0x0165;
        }
        if (receiver == "infantry" || receiver == "standard"){
            this->sender_id = 0x0103;
            this->receiver_id = 103;
        }
        else if (receiver == "hero"){
            this->receiver_id = 101;
        }
    }
    else{
        this->sender_id = 0;
        this->receiver_id = 0;
    }
}
void AbstractDraw::setColor(string color){
    this->color = color;
}
vector<uint8_t> AbstractDraw::drawLine(tuple<int, int> v1, tuple<int, int> v2){
    this->seq++;
    Draw draw;
    draw.setColor(this->color);
    vector<uint8_t> shape = draw.lineData(v1, v2);
    if (this->sender_id != 0 && this->receiver_id != 0){
        DrawMessage drawMsg = DrawMessage(this->sender_id, this->receiver_id, shape);
        Content content = Content(0x0301, &drawMsg);
        draw.setContent(&content);
        Header header = Header(&content, this->seq);
        draw.setHeader(&header);
        Footer footer = Footer(&content);
        draw.setFooter(&footer);
        return draw.createMessage();
    }
    else{
        return vector<uint8_t>();
    }
}
vector<uint8_t> AbstractDraw::drawRect(int width, int height, tuple<int, int> center){
    this->seq++;
    Draw draw;
    draw.setColor(this->color);
    vector<uint8_t> shape = draw.rectData(width, height, center);
    if (this->sender_id != 0 && this->receiver_id != 0){
        DrawMessage drawMsg = DrawMessage(this->sender_id, this->receiver_id, shape);
        Content content = Content(0x0301, &drawMsg);
        draw.setContent(&content);
        Header header = Header(&content, this->seq);
        draw.setHeader(&header);
        Footer footer = Footer(&content);
        draw.setFooter(&footer);
        return draw.createMessage();
    }
    else{
        return vector<uint8_t>();
    }
}
vector<uint8_t> AbstractDraw::drawRect(tuple<int, int> topLeft, tuple<int, int> bottomRight){
    this->seq++;
    Draw draw;
    draw.setColor(this->color);
    vector<uint8_t> shape = draw.rectData(topLeft, bottomRight);
    if (this->sender_id != 0 && this->receiver_id != 0){
        DrawMessage drawMsg = DrawMessage(this->sender_id, this->receiver_id, shape);
        Content content = Content(0x0301, &drawMsg);
        draw.setContent(&content);
        Header header = Header(&content, this->seq);
        draw.setHeader(&header);
        Footer footer = Footer(&content);
        draw.setFooter(&footer);
        return draw.createMessage();
    }
    else{
        return vector<uint8_t>();
    }
}
vector<uint8_t> AbstractDraw::drawCircle(tuple<int, int> center, int radius){
    this->seq++;
    Draw draw;
    draw.setColor(this->color);
    vector<uint8_t> shape = draw.circleData(center, radius);
    if (this->sender_id != 0 && this->receiver_id != 0){
        DrawMessage drawMsg = DrawMessage(this->sender_id, this->receiver_id, shape);
        Content content = Content(0x0301, &drawMsg);
        draw.setContent(&content);
        Header header = Header(&content, this->seq);
        draw.setHeader(&header);
        Footer footer = Footer(&content);
        draw.setFooter(&footer);
        return draw.createMessage();
    }
    else{
        return vector<uint8_t>();
    }
}
vector<uint8_t> AbstractDraw::drawText(string text, int fontSize, tuple<int, int> topLeft){
    this->seq++;
    Draw draw;
    draw.setColor(this->color);
    vector<uint8_t> shape = draw.textData(text, fontSize, topLeft);
    if (this->sender_id != 0 && this->receiver_id != 0){
        DrawMessage drawMsg = DrawMessage(this->sender_id, this->receiver_id, shape);
        Content content = Content(0x0301, &drawMsg);
        draw.setContent(&content);
        Header header = Header(&content, this->seq);
        draw.setHeader(&header);
        Footer footer = Footer(&content);
        draw.setFooter(&footer);
        return draw.createMessage();
    }
    else{
        return vector<uint8_t>();
    }
}
//int main(){
    /*//const DATA_LENGTH, CRC8, FRAME_TAIL;
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
    }*/
//}
