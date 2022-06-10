//#pragma once

#include <bitset>
#include <string>
#include <tuple>
#include <vector>

using namespace std;

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
    string color = "black";
    //index counter
    int count = 0;
    void drawWithColor() throw (exception); //uses private color to draw in color

    public:
    //Draw();
    //Draw(Header* header, Content* content, Footer* footer);
    void setColor(string color);
    string getColor();
    int getColorID();
    vector<uint8_t> lineData(tuple<int, int> v1, tuple<int, int> v2); //RETURN: array of bytes to set data_segment
    vector<uint8_t> rectData(int width, int height, tuple<int, int> center); //RETURN: array of bytes to set data_segment
    vector<uint8_t> rectData(tuple<int, int> topLeft, tuple<int, int> bottomRight); //RETURN: array of bytes to set data_segment
    vector<uint8_t> circleData(tuple<int, int> center, int radius); //RETURN: array of bytes to set data_segment
    vector<uint8_t> textData(string text, int fontSize, tuple<int, int> topLeft); // RETURN: array of bytes to set data_segment
    vector<uint8_t> createMessage(); //RETURN: array of bytes containing header, content, footer
};
