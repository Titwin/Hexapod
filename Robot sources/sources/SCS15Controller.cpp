#include "SCS15Controller.hpp"

///  default
SCS15Controller::SCS15Controller(int initialTimeout) : TTLbusController(initialTimeout)
{}
//

/// Special function (specific)
bool SCS15Controller::ping(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_PING);
    send((~(ID + messageLength + INST_PING))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return (readBuf(6) == 6);
    return false;
}
bool SCS15Controller::reset(const uint8_t& ID)
{
    const uint8_t messageLength = 2;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_RESET);
    send((~(ID + messageLength + INST_RESET))&0xFF);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return (readBuf(6) == 6);
    return false;
}
bool SCS15Controller::debug(const uint8_t& ID)
{
    const uint8_t messageLength = 4;
    const uint8_t readSize = 69;

    enableTX(true);
    printHeader(ID, messageLength);
    send(INST_READ);
    send(0);
    send(readSize);
    send((~(ID + messageLength + INST_READ + readSize))&0xFF);
    fsync(tty_fd);
    enableTX(false, messageLength);

    const uint8_t responseLength = 6+readSize;
    uint8_t buf[responseLength];
    if(readBuf(responseLength, buf) < responseLength) return false;

    std::cout << "-------Motor print-------" << std::endl;
    std::cout << "Model number: " << (int)buf[6] << '.' << (int)buf[5] << std::endl;
    std::cout << "Version: " << (int)buf[9] << '.' << (int)buf[8] << std::endl;
    std::cout << "ID: " << (int)buf[10] << std::endl;
    std::cout << "Baud rate: " << (int)buf[11] << std::endl;
    std::cout << "Return delay time: " << (int)buf[12] << std::endl;
    std::cout << "return level: " << (int)buf[13] << std::endl;
    std::cout << "Angle limit min: " << bytes2Int(buf[15], buf[14]) << std::endl;
    std::cout << "Angle limit max: " << bytes2Int(buf[17], buf[16]) << std::endl;
    std::cout << "Temperature limit: " << (int)buf[18] << std::endl;
    std::cout << "Voltage limit max: " << (int)buf[19] << std::endl;
    std::cout << "Voltage limit min: " << (int)buf[20] << std::endl;
    std::cout << "Torque max: " << bytes2Int(buf[22], buf[21]) << std::endl;
    std::cout << "Alarm led: " << (buf[23]?"yes":"no") << std::endl;
    std::cout << "Alarm shutdown: " << (int)buf[24] << std::endl;
    std::cout << "Compliance P: " << (int)buf[26] << std::endl;
    std::cout << "Compliance D: " << (int)buf[27] << std::endl;
    std::cout << "Compliance I: " << (int)buf[28] << std::endl;
    std::cout << "Punch: " << bytes2Int(buf[30], buf[29]) << std::endl;
    std::cout << "Dead zone CW: " << (int)buf[31] << std::endl;
    std::cout << "Dead zone CCW: " << (int)buf[32] << std::endl;
    std::cout << "Current max: " << bytes2Int(buf[34], buf[33]) << std::endl;
    std::cout << "Offset: " << bytes2Int(buf[36], buf[35]) << std::endl;

    std::cout << "\nTorque enable: " << (buf[45]?"true":"false") << std::endl;
    std::cout << "Led: " << (buf[46]?"yes":"no") << std::endl;
    std::cout << "Goal position: " << bytes2Int(buf[48], buf[47]) << std::endl;
    std::cout << "Goal time: " << bytes2Int(buf[50], buf[49]) << std::endl;
    std::cout << "Goal speed: " << bytes2Int(buf[52], buf[51]) << std::endl;
    std::cout << "EEPROM: " << (buf[53]?"locked":"unlocked") << std::endl;

    std::cout << "\nPosition: " << bytes2Int(buf[62], buf[61]) << std::endl;
    std::cout << "Speed: " << bytes2Int(buf[64], buf[63]) << std::endl;
    std::cout << "Load: " << bytes2Int(buf[66], buf[65]) << std::endl;
    std::cout << "Voltage: " << (int)buf[67] << std::endl;
    std::cout << "Temperature: " << (int)buf[68] << std::endl;

    //std::cout << "Registred instruction: "); << (int)buf[69]);
    //std::cout << "Error: ");               << (int)buf[5]);
    //std::cout << "Moving: ");              << (buf[71]?"yes":"no"));
    //std::cout << "Vir position: ");        << bytes2Int(buf[73], buf[72]));
    //std::cout << "Current: ");             << bytes2Int(buf[75], buf[74]));

    std::cout << "------------*------------" << std::endl;
    return true;
}
//

/// Protected functions
inline void SCS15Controller::printHeader(const uint8_t& id, const uint8_t& msgSize) const
{
    send(START_BYTE);
    send(START_BYTE);
    send(id);
    send(msgSize);
}
//


/// Set/get functions
int SCS15Controller::setRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize, const uint8_t* val)
{
    const uint8_t messageLength = 3+regSize;
    uint8_t crc = ID + messageLength + INST_WRITE + reg;

    enableTX(true);
    printHeader(ID,messageLength);
    send(INST_WRITE);
    send(reg);
    for(uint8_t i=0; i<regSize; i++)
    {
        send(val[regSize-1-i]);
        crc += val[regSize-1-i];
    }
    send((~(crc))&0xFF);
    fsync(tty_fd);
    enableTX(false, messageLength);

    if(ID != BROADCAST_ID) return readBuf(6);
    return 0;
}
int SCS15Controller::getRegister(const uint8_t& ID, const uint8_t& reg, const uint8_t& regSize)
{
    const uint8_t messageLength = 4;

    enableTX(true);
    printHeader(ID,messageLength);
    send(INST_READ);
    send(reg);
    send(regSize);
    send((~(ID + messageLength + INST_READ + reg + regSize))&0xFF);
    fsync(tty_fd);
    enableTX(false, messageLength);

    const uint8_t responseLength = 6 + regSize;
    uint8_t buf[responseLength];
    if(readBuf(responseLength, buf) < responseLength) return -1;

    if(regSize == 2) return bytes2Int(buf[6], buf[5]);
    else if(regSize == 1) return buf[5];
    else return -2;
}
void SCS15Controller::syncSetRegisterByte(const uint8_t& IDN, uint8_t* ID, const uint8_t& regStart, uint8_t* value)
{
    const uint8_t messageLength = 2*IDN+4;
    uint8_t crc = BROADCAST_ID + messageLength + INST_SYNC_WRITE + regStart + 1;

    enableTX(true);
    printHeader(BROADCAST_ID, messageLength);
    send(INST_SYNC_WRITE);
    send(regStart);
    send(1);

    for(uint8_t i=0; i<IDN; i++)
    {
        send(ID[i]);    crc += ID[i];
        send(value[i]); crc += value[i];
    }
    send((~crc)&0xFF);
    fsync(tty_fd);
    enableTX(false, messageLength);
}
void SCS15Controller::syncSetRegisterWord(const uint8_t& IDN, uint8_t* ID, const uint8_t& regStart, int* value)
{
    const uint8_t messageLength = 3*IDN+4;
    uint8_t crc = BROADCAST_ID + messageLength + INST_SYNC_WRITE + regStart + 2;

    enableTX(true);
    printHeader(BROADCAST_ID, messageLength);
    send(INST_SYNC_WRITE);
    send(regStart);
    send(2);

    uint8_t dataL, dataH;
    for(uint8_t i=0; i<IDN; i++)
    {
        int2Bytes(&dataL, &dataH, value[i]);

        send(ID[i]); crc += ID[i];
        send(dataL); crc += dataL;
        send(dataH); crc += dataH;
    }
    send((~crc)&0xFF);
    fsync(tty_fd);
    enableTX(false, messageLength);
}
int SCS15Controller::setPermanentID(const uint8_t& ID, const uint8_t& newID)
{
    if(lockEeprom(ID, false)  != 6 && ID != BROADCAST_ID) return 1;
    if(setTemporaryID(ID, newID)  != 6 && ID != BROADCAST_ID) return 2;
    if(lockEeprom(ID, true)  != 6 && ID != BROADCAST_ID) return 3;
    return 0;
}
//

