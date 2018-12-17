#include "Configuration.h"
#include "Arduino.h"

class LegBoardController
{
  public:
    //  default
    LegBoardController();
    //

    //  Special
    bool ping(uint8_t ID);
    bool reset(uint8_t ID);
    bool action(uint8_t ID);
    bool debug(uint8_t ID);
    bool autocalibrate(uint8_t ID);
    //

    //  set/get functions
    int setRegister(uint8_t ID,uint8_t reg,uint8_t regSize,uint8_t* val);
    int getRegister(uint8_t ID,uint8_t reg,uint8_t regSize);
    //void syncSetRegister(uint8_t IDN, uint8_t* ID,uint8_t regStart, uint8_t singleMsgSize, uint8_t* allRegisterValue);
    int setPermanentID(uint8_t ID, uint8_t newID);
    //
    

  public:
    //micelenious
    #define SLAVE_START 0xAA
    #define BROADCAST_ID 0xFE

    //  Status byte bitfield
    #define EEPROM_LOCK     7
    #define VLX_OK          6
    #define SERIAL_USB      5
    #define SHIELD_CONTACT  4
    #define DISTANCE_THSD   3
    #define FORCE_THSD      2
    #define LED_MODE_A      1
    #define LED_MODE_B      0
    
    #define STATUS_WRITE_MASK 0x83
    
    
    //  Register mapping
    #define REG_STATE         0
    #define REG_SHIELD        1
    #define REG_LED_RED       2
    #define REG_LED_GREEN     3
    #define REG_LED_BLUE      4
    #define REG_WDR_COUNT     5
    
    #define REG_DISTANCE_H    6
    #define REG_DISTANCE_L    7
    #define REG_FORCE_H       8
    #define REG_FORCE_L       9
    #define REG_VBATT_H       10
    #define REG_VBATT_L       11
    #define REG_TEMPERATURE_H 12
    #define REG_TEMPERATURE_L 13
    
    #define REG_SIZE          14
    
    
    #define EEPROM_ID               0
    #define EEPROM_SLAVE_TYPE       1
    #define EEPROM_SOFT_VERSION     2
    
    #define EEPROM_DISTANCE_THSD_H  3
    #define EEPROM_DISTANCE_THSD_L  4
    #define EEPROM_FORCE_THSD_H     5
    #define EEPROM_FORCE_THSD_L     6
    
    #define EEPROM_SIZE             7



  protected:
    void host2Slave(uint8_t *DataL, uint8_t* DataH, int Data);
    int  Slave2Host(uint8_t DataL, uint8_t DataH);
    int  readBuf(uint8_t len, uint8_t *buf = NULL);
    inline int lockEeprom(uint8_t ID, uint8_t enable){enable=enable?(1<<EEPROM_LOCK):0x00; return setRegister(ID, REG_STATE, 1, &enable);}
};


