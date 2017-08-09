#include "Arduino.h"
#include "Configuration.h"
#include <EEPROM.h>

class NetworkController
{
  public:
    //  default
    NetworkController();
    //

    //  init & update
    void initialize();
    void parseChar(uint8_t c);
    //


   protected:
    //  protected attributes
    uint8_t id;
    uint8_t state;
    uint8_t currentMsg[100];
    uint8_t msgLength, msgIndex, crc, msgInst, msgId;
    unsigned long lastValidMsgTimestamp;
    //

    //  network managment
    void resetState();
    void sendRegister(uint8_t regSize, uint8_t* reg);
    //

    //  responding IDs
    #define BROADCAST_ID 0xFE
    #define startByte 0xFE

    #define printHeader(id, msgSize) {Serial.write(startByte); Serial.write(startByte); Serial.write(id); Serial.write(msgSize);}
    //
    
    //  instructions
    #define INST_PING 0x01
    #define INST_READ 0x02
    #define INST_WRITE 0x03
    #define INST_REG_WRITE 0x04
    #define INST_ACTION 0x05
    #define INST_RESET 0x06
    #define INST_SYNC_WRITE 0x83
    //

    //  register Address
    #define S_ID 0x00
    #define S_SPEED_ORIENTATION 0x04
    #define S_ORIENTATION 0x05
    //
};



