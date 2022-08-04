#include <FlexCAN_T4.h>
#include <string>

//struct rmMotor_LUT----------
//desc:
'''motor-lookup table, stores information about CAN and messages'''
//var:
'''can1, can2, canReceiveMessages[][], messages[][]'''
struct rmMotor_LUT{
    // setup CAN and motor-Look-Up-Table
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

    CAN_message_t canRecieveMessages[3][11];
    CAN_message_t messages[6][2];
}

//class rmMotor----------
//desc:
'''class to store information about rmMotor'''
//var: 
'''unread, byteNum, id, canBusID, sendMsgPtr, MAX_Value, angle, rpm, torque'''
//constructor: 
'''createC620/createGM6020(rmMotor_LUT*, uint8_t, uint8_t, uint8_t): creates respective motor'''
//getters:
'''getMotors: gets all motors in an unordered map paired id:motor
   getAngle: gets angle
   getRPM: gets rpm
   getTorque: gets torque'''
//setters:
'''setPower(int): sets power'''
//others:
'''updateMotor(rmMotor_LUT*): updates the motor'''
'''updateAll: updates all motors'''
class rmMotor{
    private:
        static unordered_map<int, rmMotor*> motors;
    protected:
        bool unread_marker = false;
        short byteNum = -1;
        uint8_t id = -1
        uint8_t canBusID = -1;
        CAN_message_t* sendMsgPtr;
        short MAX_Value = -1;
        float angle = -1;
        short rpm = -1;
        short torque = -1;
    private:
        rmMotor(rmMotor_LUT* motor_LUT, uint8_t id, uint8_t motor_ID, uint8_t canBusID){
            this->byteNum = id - 1;
            this->id = motor_ID;
            this->canBusID = canBusID;
            motors[this->id] = this;
        }
        ~rmMotor(){
            motors.erase(this->id);
        }
    public:
    //constructors
        static rmMotor* createC620(rmMotor_LUT* motor_LUT, uint8_t id, uint8_t motor_ID, uint8_t canBUSID){
            rmMotor m = new rmMotor(motor_LUT, id, motor_ID, canBUSID);
            m->sendMsgPtr = &motor_LUT->messages[canBusID-1][1-byteNum/4];
            m->sendMsgPtr->id = (this->byteNum/4 >= 1)? 0x1FF:0x200;
            m->byteNum = this->byteNum%4;
            return m;
        }
        static rmMotor* createGM6020(rmMotor_LUT* motor_LUT, uint8_t id, uint8_t motor_ID, uint8_t canBUSID){
            rmMotor m = new rmMotor(motor_LUT, id, motor_ID, canBUSID);
            m->sendMsgPtr = &motor_LUT->messages[canBusID+2][1-byteNum/4];
            m->sendMsgPtr->id = (this->byteNum/4 >= 1)? 0x2FF:0x1FF;
            m->byteNum = this->byteNum%4;
            return m;
        }
    //getters
        static unordered_map<int, rmMotor*> getMotors(){
            return this->motors;
        }
        float getAngle(){
            return this.angle;
        }
        short getRPM(){
            return this.rpm;
        }
        short getTorque(){
            return this.torque;
        }
    //setters
        void setPower(float power){
            if (power > 1.0){
                power = 1.0;
            } 
            else if (power < -1.0){
                power = -1.0;
            }
  
            int16_t newPower = (int16_t)(power * 16384);
            byte byteOne = highByte(newPower);
            byte byteTwo = lowByte(newPower);
            sendMsgPtr->buf[byteNum << 1] = byteOne;
            sendMsgPtr->buf[(byteNum << 1) + 1] = byteTwo;
            updateMotor();
        }
    //others
        void updateMotor(rmMotor_LUT* motor_LUT){
            CAN_message_t* rec = &(motor_LUT->canReceiveMessages[canBusID-1][id+3]);
            this.angle = map((rec->buf[0] << 8) | rec->buf[1],0,8191,0,36000)/100.0;
            this.rpm = (rec->buf[2] << 8) | rec->buf[3];
            this.torque = (rec->buf[4] << 8) | rec->buf[5];
        }
        static void updateAll(){
            for (auto keyValuePair:rmMotor::getMotors()){
                updateMotor(keyValuePair.second);
            }
        }
}

//getMotor(int id) -> rmMotor*
'''Input: id of motor
   Output: motor pointer
'''
rmMotor* getMotor(int id){
    return rmMotor::getMotors().at(id);
}

//sendCAN(rmMotor_LUT* table)
'''Input: motor lookup table
   Output: none
   Desc: sends CAN
'''
void sendCAN(rmMotor_LUT* table){
    table->can1.write(table->messages[0][0]);
    table->can1.write(table->messages[0][1]);
    table->can1.write(table->messages[1][0]);
    table->can1.write(table->messages[1][1]);
    table->can1.write(table->messages[2][0]);
    table->can1.write(table->messages[2][1]);
    //
    table->can1.write(table->messages[3][0]);
    table->can1.write(table->messages[3][1]);
    table->can1.write(table->messages[4][0]);
    table->can1.write(table->messages[4][1]);
    table->can1.write(table->messages[5][0]);
    table->can1.write(table->messages[5][1]);
}

//readCAN(rmMotor_LUT* table)
'''Input: motor lookup table
   Output: none
   Desc: ???
'''
void readCAN(rmMotor_LUT* table){

}
