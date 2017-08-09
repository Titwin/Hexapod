#include "Configuration.h"

#include "Arduino.h"


class SlaveController
{
  public:
    //  default
    SlaveController();
    //

    //  Special
    bool ping(uint8_t ID);
    bool action(uint8_t ID);
    bool reset(uint8_t ID);
    //

    //  set/get functions
    int setRegister(uint8_t ID,uint8_t reg,uint8_t regSize,uint8_t* val);
    int getRegister(uint8_t ID,uint8_t reg,uint8_t regSize);
    float getFloatRegister(uint8_t ID, uint8_t reg);
    //
    
  protected:
    void host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);
    float bytes2Float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
    int  SCS2Host(uint8_t DataL, uint8_t DataH);
    int  readBuf(uint8_t len, uint8_t *buf = NULL);

  public:
    //micelenious
    #define Slave_startByte 0xFE
    #define BROADCAST_ID 0xFE

    //Instruction:
    #define INST_PING 0x01
    #define INST_READ 0x02
    #define INST_WRITE 0x03
    #define INST_REG_WRITE 0x04
    #define INST_ACTION 0x05
    #define INST_RESET 0x06
    #define INST_SYNC_WRITE 0x83

    //Registers
    #define S_ID 0x00
    #define S_CONFIG 0x01
    #define S_DISTANCE_1 0x02
    #define S_DISTANCE_2 0x03

    #define S_SPEED_ORIENTATION 0x04
    #define S_ORIENTATION 0x05
};


