#ifndef LEGBOARDCONTROLLER_HPP_INCLUDED
#define LEGBOARDCONTROLLER_HPP_INCLUDED

#include "TTLbusController.hpp"


class LegBoardController : public TTLbusController
{
    public:
        /// Attributes
        enum Miscellaneous
        {
        //miscellaneous
            START_BYTE = 0xAA,
            BROADCAST_ID = 0xFE,

        //instructions
            INST_PING = 0x01,
            INST_READ = 0x02,
            INST_WRITE = 0x03,
            INST_SYNC_WRITE = 0x04,
            INST_ACTION = 0x05,
            INST_RESET = 0x06,
            INST_REPLY = 0x07,
            INST_AUTOCALIBRATE = 0x08,

        //register Address
            REG_STATE = 0,
            REG_SHIELD = 1,
            REG_RED = 2,
            REG_GREEN = 3,
            REG_BLUE = 4,
            REG_WDR_COUNT = 5,

            REG_DISTANCE_H = 6,       REG_DISTANCE_L = 7,
            REG_FORCE_H = 8,          REG_FORCE_L = 9,
            REG_VBATT_H = 10,         REG_VBATT_L = 11,
            REG_TEMPERATURE_H = 12,   REG_TEMPERATURE_L = 13,

            REG_SIZE = 14,

            EEPROM_ID = 0,
            EEPROM_SLAVE_TYPE = 1,
            EEPROM_SOFT_VERSION = 2,
            EEPROM_DISTANCE_THSD_H = 3,  EEPROM_DISTANCE_THSD_L = 4,
            EEPROM_FORCE_THSD_H = 5,     EEPROM_FORCE_THSD_L = 6,

            EEPROM_SIZE = 7,

        //  Status bit-field
            EEPROM_LOCK = 7,
            VLX_OK = 6,
            SERIAL_USB = 5,
            SHIELD_CONTACT = 4,
            DISTANCE_THSD = 3,
            FORCE_THSD = 2,
            LED_MODE_A = 1,
            LED_MODE_B = 0,

            STATUS_WRITE_MASK = 0x83
        };
        //

        ///  Default
        LegBoardController(int initialTimeout = 3);
        //

        ///  Special
        int scan(uint8_t* idSize, uint8_t* idList);
        bool ping(const uint8_t& ID);
        bool action(const uint8_t& ID);
        bool reset(const uint8_t& ID);
        bool debug(const uint8_t& ID, uint8_t* response = NULL);
        bool autocalibrate(const uint8_t& ID);
        inline uint8_t getType(const uint8_t& ID){return getRegister(ID, EEPROM_SLAVE_TYPE, 1);};
        //

        ///  set/get functions
        int setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, const uint8_t* const val);
        int getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize);
        bool readMemory(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, uint8_t* response);
        //

    protected:
        /// Protected functions
        inline void printHeader(const uint8_t& id, const uint8_t& msgSize) const;
        //


        //

    public:
        /// Useful functions
        inline int lockEeprom(const uint8_t& ID, uint8_t enable){ enable = enable?(1<<EEPROM_LOCK):0x00; return setRegister(ID, REG_STATE, 1, &enable); }
        inline int setTemporaryID(const uint8_t& ID, const uint8_t& newID) { return setRegister(ID, REG_SIZE + EEPROM_ID, 1, &newID); }
        int setPermanentID(const uint8_t& ID, const uint8_t& newID);

        int setColor(const uint8_t& ID, const uint8_t& red, const uint8_t& green, const uint8_t& blue);
        inline int setBlinkMode(const uint8_t& ID, uint8_t mode) { mode = (mode&0x03)|(1<<EEPROM_LOCK); return setRegister(ID, REG_STATE, 1, &mode); };

        inline int getState(const uint8_t& ID) { return getRegister(ID, REG_STATE, 1); };
        inline int getDistance(const uint8_t& ID) { return getRegister(ID, REG_DISTANCE_H, 2); };
        inline int getForce(const uint8_t& ID) { return getRegister(ID, REG_FORCE_H, 2); };
        inline int getShield(const uint8_t& ID) { return getRegister(ID, REG_SHIELD, 1); };
        uint32_t getColor(const uint8_t& ID);
        //
};

#endif // LEGBOARDCONTROLLER_HPP_INCLUDED
