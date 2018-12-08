#include "Arduino.h"

//  network constant macros
#define NC_BROADCAST_ID     0xFE
#define NC_START_BYTE       0xFE

#define NC_INST_PING            0x01
#define NC_INST_ACTION          0x05
#define NC_INST_READ            0x02
#define NC_INST_WRITE           0x03
#define NC_INST_REG_WRITE       0x04
#define NC_INST_RESET           0x06
#define NC_INST_REPLY           0x07
#define NC_INST_AUTOCALIBRATE   0x08


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
    bool isPersonalId(const uint8_t& Id = 0xFF);
    void setId(const uint8_t& newid);
    //

    //  ISR vector interrupt
    void rx_complete_irq();
    //

  protected:
    //  protected attributes
    uint8_t id;
    uint8_t state;
    uint8_t currentMsg[20];
    uint8_t msgLength, msgIndex, crc, msgInst, msgId;
    unsigned long lastValidCharTimestamp;
    NetworkCallback callbackArray[7];

    volatile uint8_t _rx_buffer_head;
    volatile uint8_t _rx_buffer_tail;
    volatile uint8_t _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    //

    //  network managment
    void resetState();
    void inline printHeader(const uint8_t& id, const uint8_t& msgSize, const uint8_t& start = NC_START_BYTE);
    uint8_t inline write(uint8_t c);
    int read();
    int available();
    //
};

