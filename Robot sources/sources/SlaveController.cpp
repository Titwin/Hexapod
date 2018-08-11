#include "SlaveController.hpp"


///  Default
SlaveController::SlaveController(int initialTimeout) : TTLbusController(initialTimeout)
{}
//

///  Special
bool SlaveController::ping(const uint8_t& ID)
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
bool SlaveController::action(const uint8_t& ID)
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
bool SlaveController::reset(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_RESET);
    send((~(ID + messageLength + INST_RESET))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
bool SlaveController::debug(const uint8_t& ID)
{
    const uint8_t messageLength = 4;
    const uint8_t readSize = 14;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_READ);
    send(0);
    send(readSize);
    send((~(ID + messageLength + INST_READ + readSize))&0xFF);
    enableTX(false, messageLength);

    const uint8_t responseLength = 6+readSize;
    uint8_t buf[responseLength];
    if(readBuf(responseLength, buf) < responseLength) return false;

    std::cout <<"-------Slave print-------" << std::endl;
    std::cout << "State: 0x" << std::hex << (int)buf[5] << std::dec << std::endl;
    std::cout << "  EEPROM: " << (buf[5]&(1 << 7)?"lock":"unlock") << std::endl;
    std::cout << "  Distance sensor: " << (buf[5]&(1 << 6)?"ready":"error") << std::endl;
    std::cout << "  USB: " << (buf[5]&(1 << 5)?"connected":"disconnected") << std::endl;
    std::cout << "  Shield contact: " << (buf[5]&(1 << 4)?"yes":"no") << std::endl;
    std::cout << "  Closed distance: " << (buf[5]&(1 << 3)?"yes":"no") << std::endl;
    std::cout << "  Touching: " << (buf[5]&(1 << 2)?"yes":"no") << std::endl;
    std::cout << "  Blink mode: " << (int)(buf[5]&0x03) << std::endl;

    std::cout << "Distance: " << bytes2Int(buf[7], buf[6]) << std::endl;
    std::cout << "Force: " << bytes2Int(buf[9], buf[8]) << std::endl;
    std::cout << "Shield: 0x" << std::hex << (int)buf[10] << std::dec << std::endl;
    std::cout << "  A: " << (buf[10]&(1 << 0)?"contact":"0") << std::endl;
    std::cout << "  B: " << (buf[10]&(1 << 2)?"contact":"0") << std::endl;
    std::cout << "  C: " << (buf[10]&(1 << 1)?"contact":"0") << std::endl;
    std::cout << "  D: " << (buf[10]&(1 << 3)?"contact":"0") << std::endl;
    std::cout << "  center: " << (buf[10]&(1 << 4)?"contact":"0") << std::endl;

    std::cout << "Color :" << std::endl;
    std::cout << "  red: " << (int)buf[11] << std::endl;
    std::cout << "  green: " << (int)buf[12] << std::endl;
    std::cout << "  blue: " << (int)buf[13] << std::endl;

    std::cout << "Id: " << (int)buf[14] << std::endl;
    std::cout << "Distance threshold: " << bytes2Int(buf[16], buf[15]) << std::endl;
    std::cout << "Force threshold: " << bytes2Int(buf[18], buf[17]) << std::endl;

    std::cout << "------------*------------" << std::endl;
    return true;
}
//

///  Private functions
inline void SlaveController::printHeader(const uint8_t& id, const uint8_t& msgSize) const
{
    send(START_BYTE);
    send(START_BYTE);
    send(id);
    send(msgSize);
}
//

///  Set/get functions
int SlaveController::setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, uint8_t* val)
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
int SlaveController::getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize)
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

    if(regSize == 2) return bytes2Int(buf[6], buf[5]);
    else if(regSize == 1) return buf[5];
    else return -2;
}
float SlaveController::getFloatRegister(const uint8_t& ID, const uint8_t& reg)
{
    const uint8_t messageLength = 4;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_READ);
    send(reg);
    send(4);
    send((~(ID + messageLength + INST_READ + reg + 4))&0xFF);
    enableTX(false, messageLength);

    uint8_t buf[10];
    if(readBuf(10, buf) < 10) return -4.2e12;
    return bytes2Float(buf[8],buf[7],buf[6],buf[5]);
}
//
