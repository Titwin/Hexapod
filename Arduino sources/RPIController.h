#include "Arduino.h"
#include "Configuration.h"

class RPIController
{
  public:    
    //  default
    RPIController();

    void initialize();
    //

    //  trame parser
    void update(unsigned long stoptime);
    void sendMsg(uint8_t code,uint8_t dataLength,uint8_t* data,uint8_t target2 = 0);
    //
    
  protected:
    //  protected functions
    void resetState();
    void processMsg();
    void parseChar(uint8_t c);
    //
    
  public:
    uint8_t state;
    uint8_t currentMsg[100];
    uint8_t msgLength, msgIndex, crc, msgCode;
    unsigned long lastValidMsgTimestamp;
    
    //  miscellaneous
    #define startByte 0xFF
    //

    //  instruction
    #define RPI_INST_SHIFT  4
    #define RPI_INST_MASK   (0x0F<<RPI_INST_SHIFT)

    #define RPI_INST_PING   (1<<RPI_INST_SHIFT)
    #define RPI_INST_READ   (2<<RPI_INST_SHIFT)
    #define RPI_INST_WRITE  (3<<RPI_INST_SHIFT)
    #define RPI_INST_ACK    (4<<RPI_INST_SHIFT)
    //



  protected:
    inline void sendAck(){sendMsg(RPI_INST_ACK, 0, NULL);};
    
  public:
    inline void sendPing(){sendMsg(RPI_INST_PING, 0, NULL);};
};

