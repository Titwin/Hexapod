#ifndef SLAVECONTROLLER_HPP_INCLUDED
#define SLAVECONTROLLER_HPP_INCLUDED

#include "TTLbusController.hpp"


class SlaveController : private TTLbusController
{
    public:
        ///  Default
        SlaveController(int initialTimeout = 5);
        //

        ///  Special
        bool ping(const uint8_t& ID);
        bool action(const uint8_t& ID);
        bool reset(const uint8_t& ID);
        bool debug(const uint8_t& ID);
        //

        ///  set/get functions
        int setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, uint8_t* val);
        int getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize);
        float getFloatRegister(const uint8_t& ID, const uint8_t& reg);
        //

    protected:
        /// Protected functions
        inline void printHeader(const uint8_t& id, const uint8_t& msgSize) const;
        //

        /// Attributes
        enum Miscellaneous
        {
        //miscellaneous
            START_BYTE = 0xFE,
            BROADCAST_ID = 0xFE,

        //instructions
            INST_PING = 0x01,
            INST_READ = 0x02,
            INST_WRITE = 0x03,
            INST_REG_WRITE = 0x04,
            INST_ACTION = 0x05,
            INST_RESET = 0x06,
            INST_SYNC_WRITE = 0x83,

        //register Address
            S_ID = 0x00,
            S_CONFIG = 0x01,
            S_DISTANCE_1 = 0x02,
            S_DISTANCE_2 = 0x03,

            S_SPEED_ORIENTATION = 0x04,
            S_ORIENTATION = 0x05,
        };
        //
};

#endif // SLAVECONTROLLER_HPP_INCLUDED
