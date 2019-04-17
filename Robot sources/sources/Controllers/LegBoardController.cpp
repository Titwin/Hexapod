#include "LegBoardController.hpp"


///  Default
LegBoardController::LegBoardController(int initialTimeout) : TTLbusController(initialTimeout)
{}
//

///  Special
int LegBoardController::scan(uint8_t* idSize, uint8_t* idList)
{
    int ackcount = 0;
    if(idSize) *idSize = 0;
    for(uint8_t i=0; i<BROADCAST_ID; i++)
    {
        if(ping(i))
        {
            ackcount++;
            if(idList && idSize)
            {
                idList[*idSize] = i;
                (*idSize)++;
            }
        }
    }
    return ackcount;
}
bool LegBoardController::ping(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_PING);
    send((~(ID + messageLength + INST_PING))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
bool LegBoardController::action(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_ACTION);
    send((~(ID + messageLength + INST_ACTION))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
bool LegBoardController::reset(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_AUTOCALIBRATE);
    send((~(ID + messageLength + INST_RESET))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
bool LegBoardController::autocalibrate(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_RESET);
    send((~(ID + messageLength + INST_RESET))&0xFF);
    enableTX(false, messageLength);

    int t = getTimeout();
    setTimeout(500);
    bool error = false;
    if(ID != BROADCAST_ID) error = (readBuf(6) == 6);
    setTimeout(t);
    return error;
}
bool LegBoardController::debug(const uint8_t& ID, uint8_t* response)
{
    const uint8_t readSize = REG_SIZE + EEPROM_SIZE;

    if(!response)
    {
        const uint8_t responseLength = 6+readSize;
        uint8_t buf[responseLength];
        if(!readMemory(ID, 0, readSize, buf)) return false;

        std::cout <<"-------LegBoard print-------" << std::endl;
        uint8_t tmp = buf[5 + REG_STATE];
        std::cout << "State: 0x" << std::hex << (int)tmp << std::dec << std::endl;
        std::cout << "  EEPROM: " << (tmp&(1 << 7)?"lock":"unlock") << std::endl;
        std::cout << "  Distance sensor: " << (tmp&(1 << 6)?"ready":"error") << std::endl;
        std::cout << "  USB: " << (tmp&(1 << 5)?"connected":"disconnected") << std::endl;
        std::cout << "  Shield contact: " << (tmp&(1 << 4)?"yes":"no") << std::endl;
        std::cout << "  Proximity alert: " << (tmp&(1 << 3)?"yes":"no") << std::endl;
        std::cout << "  Touching: " << (tmp&(1 << 2)?"yes":"no") << std::endl;
        std::cout << "  Blink mode: " << (int)(tmp&0x03) << std::endl;

        tmp = buf[5 + REG_SHIELD];
        std::cout << "Shield: 0x" << std::hex << (int)tmp << std::dec << std::endl;
        std::cout << "  A: " << (tmp&(1 << 0)?"contact":"0") << std::endl;
        std::cout << "  B: " << (tmp&(1 << 2)?"contact":"0") << std::endl;
        std::cout << "  C: " << (tmp&(1 << 1)?"contact":"0") << std::endl;
        std::cout << "  D: " << (tmp&(1 << 3)?"contact":"0") << std::endl;
        std::cout << "  center: " << (tmp&(1 << 4)?"contact":"0") << std::endl;

        std::cout << "Color :" << std::endl;
        std::cout << "  red: " << (int)buf[5 + REG_RED] << std::endl;
        std::cout << "  green: " << (int)buf[5 + REG_GREEN] << std::endl;
        std::cout << "  blue: " << (int)buf[5 + REG_BLUE] << std::endl;

        std::cout << "Deadlocks: " << (int)buf[5 + REG_WDR_COUNT] << std::endl;

        std::cout << "Distance: " << bytes2Int(buf[5 + REG_DISTANCE_L], buf[5 + REG_DISTANCE_H]) << std::endl;
        std::cout << "   threshold: " << bytes2Int(buf[5 + REG_SIZE + EEPROM_DISTANCE_THSD_L], buf[5 + REG_SIZE + EEPROM_DISTANCE_THSD_H]) << std::endl;
        std::cout << "Force: " << bytes2Int(buf[5 + REG_FORCE_L], buf[5 + REG_FORCE_H]) << std::endl;
        std::cout << "   threshold: " << bytes2Int(buf[5 + REG_SIZE + EEPROM_FORCE_THSD_L], buf[5 + REG_SIZE + EEPROM_FORCE_THSD_H]) << std::endl;
        std::cout << "Voltage: " << bytes2Int(buf[5 + REG_VBATT_L], buf[5 + REG_VBATT_H]) << std::endl;
        std::cout << "Temperature: " << bytes2Int(buf[5 + REG_TEMPERATURE_L], buf[5 + REG_TEMPERATURE_H]) << std::endl;

        std::cout << "Id: " << (int)buf[5 + REG_SIZE + EEPROM_ID] << std::endl;
        std::cout << "Slave type: " << (int)buf[5 + REG_SIZE + EEPROM_SLAVE_TYPE] << std::endl;
        std::cout << "Firmware version: " << (int)buf[5 + REG_SIZE + EEPROM_SOFT_VERSION] << std::endl;

        std::cout << "------------*------------" << std::endl;
        return true;
    }
    else return readMemory(ID, 0, readSize, response);
}
//

///  Private functions
inline void LegBoardController::printHeader(const uint8_t& id, const uint8_t& msgSize) const
{
    send(START_BYTE);
    send(START_BYTE);
    send(id);
    send(msgSize);
}
//

///  Set/get functions
int LegBoardController::setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, const uint8_t* const val)
{
    const uint8_t messageLength = 3+regSize;
    uint8_t crc = ID + messageLength + INST_WRITE + reg;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_WRITE);
    send(reg);
    for(uint8_t i=0; i<regSize; i++)
    {
        send(val[regSize-1-i]);
        crc += val[regSize-1-i];
    }
    send((~(crc))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
int LegBoardController::getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize)
{
    const uint8_t messageLength = 4;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_READ);
    send(reg);
    send(regSize);
    send((~(ID + messageLength + INST_READ + reg + regSize))&0xFF);
    enableTX(false, messageLength);

    const uint8_t responseLength = 6 + regSize;
    uint8_t buf[responseLength];
    if(readBuf(responseLength, buf) < responseLength) return -1;

    if(regSize == 2) return bytes2Int(buf[5], buf[6]);
    else if(regSize == 1) return buf[5];
    else return -2;
}
bool LegBoardController::readMemory(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, uint8_t* response)
{
    if(!response) return false;

    const uint8_t messageLength = 4;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_READ);
    send(reg);
    send(regSize);
    send((~(ID + messageLength + INST_READ + reg + regSize))&0xFF);
    enableTX(false, messageLength);

    const uint8_t responseLength = 6+regSize;

    if(readBuf(responseLength, response) < responseLength) return false;
    return true;
}
//

/// Useful functions
int LegBoardController::setPermanentID(const uint8_t& ID, const uint8_t& newID)
{
    int t = getTimeout();
    setTimeout(20);
    int error = 0;
    for(;;)
    {
        error = (lockEeprom(ID, false)!= 6 && ID != BROADCAST_ID)?1:0;     if(error) break;
        error = (setTemporaryID(ID, newID)!= 6 && ID != BROADCAST_ID)?2:0; if(error) break;
        error = (lockEeprom(newID, true)!= 6 && ID != BROADCAST_ID)?3:0;   if(error) break;
        break;
    }
    setTimeout(t);
    return error;
}
int LegBoardController::setColor(const uint8_t& ID, const uint8_t& red, const uint8_t& green, const uint8_t& blue)
{
    int error = 0;
    for(;;)
    {
        error = (setRegister(ID, REG_RED, 1, &red)!= 6 && ID != BROADCAST_ID)?1:0;     if(error) break;
        error = (setRegister(ID, REG_GREEN, 1, &green)!= 6 && ID != BROADCAST_ID)?2:0; if(error) break;
        error = (setRegister(ID, REG_BLUE, 1, &blue)!= 6 && ID != BROADCAST_ID)?3:0;   if(error) break;
        break;
    }
    return error;
}
uint32_t LegBoardController::getColor(const uint8_t& ID)
{
    int red = getRegister(ID, REG_RED, 1);
    int green = getRegister(ID, REG_GREEN, 1);
    int blue = getRegister(ID, REG_BLUE, 1);

    if(red >= 0 && green >= 0 && blue >= 0)
        return (red<<16) | (green<<8) | blue;
    else return -1;
}
//
