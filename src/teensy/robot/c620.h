// #include "rmMotor.h"

class c620 {
    private:
        short id;
        CAN_message_t& sendMsg;
        int byteNum;
    public:
        c620(short tempID, CAN_message_t* msg);
        void setPower(float power);
    
};
