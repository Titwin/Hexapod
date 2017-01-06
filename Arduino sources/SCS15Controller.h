#include "Arduino.h"
#include "Configuration.h"

class SCS15Controller
{
  public:
    //  default
    SCS15Controller();
    //

    //  Special
    void initialize();
    bool ping(uint8_t ID);
    void debug(uint8_t ID);
    //

    //  set/get functions
    int setRegister(uint8_t ID,uint8_t reg,uint8_t regSize,uint8_t* val);
    int getRegister(uint8_t ID,uint8_t reg,uint8_t regSize);
    void syncSetRegister(uint8_t IDN, uint8_t* ID,uint8_t regStart, uint8_t singleMsgSize, uint8_t* allRegisterValue);
    //
    
  protected:
    void host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);
    int  SCS2Host(uint8_t DataL, uint8_t DataH);
    int  readBuf(uint8_t len, uint8_t *buf = NULL);

  public:
    //micelenious
    #define startByte 0xFF
    #define BROADCAST_ID 0xFE

    //register Address
    #define P_MODEL_NUMBER_L 0
    #define P_MODEL_NUMBER_H 1
    #define P_VERSION_L 3   
    #define P_VERSION_H 4
    #define P_ID 5
    #define P_BAUD_RATE 6
    #define P_RETURN_DELAY_TIME 7
    #define P_RETURN_LEVEL 8
    #define P_MIN_ANGLE_LIMIT_L 9
    #define P_MIN_ANGLE_LIMIT_H 10
    #define P_MAX_ANGLE_LIMIT_L 11
    #define P_MAX_ANGLE_LIMIT_H 12
    #define P_LIMIT_TEMPERATURE 13
    #define P_MAX_LIMIT_VOLTAGE 14
    #define P_MIN_LIMIT_VOLTAGE 15
    #define P_MAX_TORQUE_L 16
    #define P_MAX_TORQUE_H 17
    #define P_ALARM_LED 18
    #define P_ALARM_SHUTDOWN 19
    #define P_COMPLIANCE_P 21
    #define P_COMPLIANCE_D 22
    #define P_COMPLIANCE_I 23
    #define P_PUNCH_L 24
    #define P_PUNCH_H 25
    #define P_CW_DEAD 26
    #define P_CCW_DEAD 27
    #define P_IMAX_L 28
    #define P_IMAX_H 29
    #define P_OFFSET_L 30
    #define P_OFFSET_H 31

    #define P_TORQUE_ENABLE 40
    #define P_LED 41
    #define P_GOAL_POSITION_L 42
    #define P_GOAL_POSITION_H 43
    #define P_GOAL_TIME_L 44
    #define P_GOAL_TIME_H 45
    #define P_GOAL_SPEED_L 46
    #define P_GOAL_SPEED_H 47
    #define P_LOCK 48

    #define P_PRESENT_POSITION_L 56
    #define P_PRESENT_POSITION_H 57
    #define P_PRESENT_SPEED_L 58
    #define P_PRESENT_SPEED_H 59
    #define P_PRESENT_LOAD_L 60
    #define P_PRESENT_LOAD_H 61
    #define P_PRESENT_VOLTAGE 62
    #define P_PRESENT_TEMPERATURE 63
    #define P_REGISTERED_INSTRUCTION 64
    #define P_ERROR 65
    #define P_MOVING 66
    #define P_VIR_POSITION_L 67
    #define P_VIR_POSITION_H 68
    #define P_CURRENT_L 69
    #define P_CURRENT_H 70

    //Instruction:
    #define INST_PING 0x01
    #define INST_READ 0x02
    #define INST_WRITE 0x03
    #define INST_REG_WRITE 0x04
    #define INST_ACTION 0x05
    #define INST_RESET 0x06
    #define INST_SYNC_WRITE 0x83

    inline int enableTorque(uint8_t ID, uint8_t enable) {return setRegister(ID, P_TORQUE_ENABLE, 1, &enable);}
    inline int setLimitTroque(uint8_t ID, int maxTroque){return setRegister(ID, P_MAX_TORQUE_L, 2, (uint8_t*)&maxTroque);}
    inline int setSpeed(uint8_t ID, int speed)          {return setRegister(ID, P_GOAL_TIME_L, 2, (uint8_t*)&speed);}

    inline int getEnableTorque(uint8_t ID){return getRegister(ID, P_TORQUE_ENABLE, 1);}
    inline int getTorque(uint8_t ID)      {return getRegister(ID, P_PRESENT_LOAD_L, 2);}
    inline int getTemperature(uint8_t ID) {return getRegister(ID, P_PRESENT_TEMPERATURE, 1);}
    inline int getPosition(uint8_t ID)    {return getRegister(ID, P_PRESENT_POSITION_L, 2);}

    inline void syncSetPosition(uint8_t ID[], uint8_t IDN, int position[]) {syncSetRegister(IDN, ID, P_GOAL_POSITION_L, 2, (uint8_t*)position);}
};


