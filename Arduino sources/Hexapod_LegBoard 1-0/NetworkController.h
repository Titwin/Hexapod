#include "Arduino.h"

//  network constant macros
#define NC_BROADCAST_ID     0xFE
#define NC_START_BYTE       0xFE

#define NC_INST_PING        0x01
#define NC_INST_ACTION      0x05
#define NC_INST_READ        0x02
#define NC_INST_WRITE       0x03
#define NC_INST_REG_WRITE   0x04
#define NC_INST_RESET       0x06
#define NC_INST_REPLY       0x07









class NetworkController
{
  public:
    //  Miscellaneous
    typedef void (*NetworkCallback)(uint8_t msgSize, const uint8_t* msg);
    //
    
    //  default
    NetworkController();
    //

    //  init & update
    void update();
    void initialize(uint8_t startingId = 0x01);
    void parseChar(uint8_t c);
    //

    //  public functions
    bool assignCallback(uint8_t instructionType, NetworkCallback callback);
    void sendRegister(const uint8_t& instruction, const uint8_t& regSize, const uint8_t* reg);
    unsigned long getLastTimestamp();
    bool isPersonalId();
    void setId(const uint8_t& newid);
    //

  protected:
    //  protected attributes
    uint8_t id;
    uint8_t state;
    uint8_t currentMsg[20];
    uint8_t msgLength, msgIndex, crc, msgInst, msgId;
    unsigned long lastValidCharTimestamp;
    NetworkCallback callbackArray[6];
    //

    //  network managment
    void resetState();
    void inline printHeader(const uint8_t& id, const uint8_t& msgSize, const uint8_t& start = NC_START_BYTE);
    //
};



