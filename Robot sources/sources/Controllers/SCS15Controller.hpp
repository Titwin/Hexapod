#ifndef SCS15CONTROLLER_HPP_INCLUDED
#define SCS15CONTROLLER_HPP_INCLUDED

#include "TTLbusController.hpp"


class SCS15Controller : public TTLbusController
{
    public:
        /// Attributes
        enum Miscellaneous
        {
        //miscellaneous
            START_BYTE = 0xFF,
            BROADCAST_ID = 0xFE,

        //register Address
            P_MODEL_NUMBER_L = 0,           P_MODEL_NUMBER_H = 1,
            P_VERSION_L = 3,                P_VERSION_H = 4,
            P_ID = 5,
            P_BAUD_RATE = 6,
            P_RETURN_DELAY_TIME = 7,
            P_RETURN_LEVEL = 8,
            P_MIN_ANGLE_LIMIT_L = 9,        P_MIN_ANGLE_LIMIT_H = 10,
            P_MAX_ANGLE_LIMIT_L = 11,       P_MAX_ANGLE_LIMIT_H = 12,
            P_LIMIT_TEMPERATURE = 13,
            P_MAX_LIMIT_VOLTAGE = 14,
            P_MIN_LIMIT_VOLTAGE = 15,
            P_MAX_TORQUE_L = 16,            P_MAX_TORQUE_H = 17,
            P_ALARM_LED = 18,
            P_ALARM_SHUTDOWN = 19,
            P_COMPLIANCE_P = 21,
            P_COMPLIANCE_D = 22,
            P_COMPLIANCE_I = 23,
            P_PUNCH_L = 24,                 P_PUNCH_H = 25,
            P_CW_DEAD = 26,
            P_CCW_DEAD = 27,
            P_IMAX_L = 28,                  P_IMAX_H = 29,
            P_OFFSET_L = 30,                P_OFFSET_H = 31,

            P_TORQUE_ENABLE = 40,
            P_LED = 41,
            P_GOAL_POSITION_L = 42,         P_GOAL_POSITION_H = 43,
            P_GOAL_TIME_L = 44,             P_GOAL_TIME_H = 45,
            P_GOAL_SPEED_L = 46,            P_GOAL_SPEED_H = 47,
            P_LOCK = 48,

            P_PRESENT_POSITION_L = 56,      P_PRESENT_POSITION_H = 57,
            P_PRESENT_SPEED_L = 58,         P_PRESENT_SPEED_H = 59,
            P_PRESENT_LOAD_L = 60,          P_PRESENT_LOAD_H = 61,
            P_PRESENT_VOLTAGE = 62,
            P_PRESENT_TEMPERATURE = 63,
            P_REGISTERED_INSTRUCTION = 64,
            P_ERROR = 65,
            P_MOVING = 66,
            P_VIR_POSITION_L = 67,          P_VIR_POSITION_H = 68,
            P_CURRENT_L = 69,               P_CURRENT_H = 70,

        //Instruction:
            INST_PING = 0x01,
            INST_READ = 0x02,
            INST_WRITE = 0x03,
            INST_REG_WRITE = 0x04,
            INST_ACTION = 0x05,
            INST_RESET = 0x06,
            INST_SYNC_WRITE = 0x83
        };
        //

        ///  default
        SCS15Controller(int initialTimeout = 3);
        //

        /// Set/get functions
        int setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, const uint8_t* val);
        int getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize);
        void syncSetRegisterByte(const uint8_t& IDN, uint8_t* ID, const uint8_t& regStart, uint8_t* value);
        void syncSetRegisterWord(const uint8_t& IDN, uint8_t* ID, const uint8_t& regStart, uint16_t* value);
        //

        /// Special function (specific)
        int scan(uint8_t* idSize, uint8_t* idList);
        bool ping(const uint8_t& ID);
        bool reset(const uint8_t& ID);
        bool debug(const uint8_t& ID, uint8_t* response = NULL);
        //

    protected:
        /// Protected functions
        inline void printHeader(const uint8_t& id, const uint8_t& msgSize) const;
        //

    public:
        /// Special function (specific)
        inline int lockEeprom(const uint8_t& ID, const uint8_t& enable)    {return setRegister(ID, P_LOCK, 1, &enable);}
        inline int setTemporaryID(const uint8_t& ID, const uint8_t& newID) {return setRegister(ID, P_ID, 1, &newID);}
        int setPermanentID(const uint8_t& ID, const uint8_t& newID);

        //inline int enableTorque(const uint8_t& ID, const uint8_t& enable)       {return setRegister(ID, P_TORQUE_ENABLE, 1, &enable);}
        //inline int setLimitTroque(const uint8_t& ID, const uint16_t& maxTroque) {return setRegister(ID, P_MAX_TORQUE_L, 2, (uint8_t*)&maxTroque);}
        //inline int setSpeed(const uint8_t& ID, const uint16_t& speed)           {return setRegister(ID, P_GOAL_TIME_L, 2, (uint8_t*)&speed);}
        //inline int setPosition(const uint8_t& ID, const uint16_t& position)     {return setRegister(ID, P_GOAL_POSITION_L, 2, (uint8_t*)&position);}

        inline int getEnableTorque(const uint8_t& ID){return getRegister(ID, P_TORQUE_ENABLE, 1);}
        inline int getTorque(const uint8_t& ID)      {return getRegister(ID, P_PRESENT_LOAD_L, 2);}
        inline int getTemperature(const uint8_t& ID) {return getRegister(ID, P_PRESENT_TEMPERATURE, 1);}
        inline int getPosition(const uint8_t& ID)    {return getRegister(ID, P_PRESENT_POSITION_L, 2);}

        inline void syncSetPosition(uint8_t* ID, const uint8_t& IDN, uint16_t* pos) {syncSetRegisterWord(IDN, ID, P_GOAL_POSITION_L, pos);}
        inline void syncSetTorque(uint8_t* ID, const uint8_t& IDN, uint8_t* torque) {syncSetRegisterByte(IDN, ID, P_TORQUE_ENABLE, torque);}
        //
};

#endif // SCS15CONTROLLER_HPP_INCLUDED
