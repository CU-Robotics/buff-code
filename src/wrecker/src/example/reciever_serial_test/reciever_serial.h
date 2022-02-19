#define REC_SERIAL_H

struct reciever_message
{
  int rStickH;
  int rStickV;
  int lStickH;
  int lStickV;
  byte s1;
  byte s2;
  int mouseX;
  int mouseY;
  int mouseZ;
  byte rightButton;
  byte leftButton;
  bool w;
  bool s;
  bool a;
  bool d;
  bool shift;
  bool ctrl;
  bool q;
  bool e;
  bool r;
  bool f;
  bool g;
  bool z;
  bool x;
  bool c;
  bool v;
  bool space;
};

class recSerial {
    private:
        reciever_message message;
    public:
        recSerial();
        bool parseMessage(byte buf[14]);     //expects a 14 byte array
        int getRightStickHorizontal();
        int getRightStickVertical();
        int getLeftStickHorizontal();
        int getLeftStickVertical();
        byte getSwitchOne();
        byte getSwitchTwo();
        int getMouseX();
        int getMouseY();
        int getMouseZ();
        bool rightMousePressed();
        bool leftMousePressed();
        bool keyPressed(String key);
}