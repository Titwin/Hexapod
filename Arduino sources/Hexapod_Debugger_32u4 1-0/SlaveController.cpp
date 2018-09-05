#include "SlaveController.h"

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_SYNC_WRITE 0x83

//  Default
SlaveController::SlaveController(){}
//

//  private functions
int SlaveController::readBuf(uint8_t len, uint8_t *buf)
{
  if(buf) return scanf(buf,len);
  else
  {
    uint8_t buftmp[10];
    return scanf(buftmp,len);
  }
}
int SlaveController::Slave2Host(uint8_t DataL, uint8_t DataH)
{
  return ((int)DataH<<8)|DataL;
}
void SlaveController::host2Slave(uint8_t *DataL, uint8_t* DataH, int Data)
{
  *DataL = (Data>>8);
  *DataH = (Data&0xFF);
}
//


//  Special
bool SlaveController::ping(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(SLAVE_START, ID,messageLength);
  printf(INST_PING);
  printf((~(ID + messageLength + INST_PING))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == 6);
  return false;
}
bool SlaveController::reset(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(SLAVE_START, ID,messageLength);
  printf(INST_RESET);
  printf((~(ID + messageLength + INST_RESET))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == 6);
  return false;
}
bool SlaveController::action(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(SLAVE_START, ID,messageLength);
  printf(INST_ACTION);
  printf((~(ID + messageLength + INST_ACTION))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == 6);
  return false;
}
bool SlaveController::debug(uint8_t ID)
{
  const uint8_t messageLength = 4;
  const uint8_t readSize = 16;

  disableRX();
  printHeader(SLAVE_START, ID, messageLength);
  printf(INST_READ);
  printf(0);
  printf(readSize);
  printf((~(ID + messageLength + INST_READ + readSize))&0xFF);
  flush();
  enableRX();

  const uint8_t responseLength = 6+readSize;
  uint8_t buf[responseLength];
  if(readBuf(responseLength, buf) < responseLength) return false;

  Serial.println(F("-------Slave print-------"));
  Serial.print(F("State: 0x"));            Serial.println(buf[5], HEX);
  Serial.print(F("  EEPROM: "));           Serial.println((buf[5]&(1<<7)?"lock":"unlock"));
  Serial.print(F("  Distance sensor: "));  Serial.println((buf[5]&(1<<6)?"ready":"error"));
  Serial.print(F("  USB: "));              Serial.println((buf[5]&(1<<5)?"connected":"disconnected"));
  Serial.print(F("  Shield contact: "));   Serial.println((buf[5]&(1<<4)?"yes":"no"));
  Serial.print(F("  Closed distance: "));  Serial.println((buf[5]&(1<<3)?"yes":"no"));
  Serial.print(F("  Touching: "));         Serial.println((buf[5]&(1<<2)?"yes":"no"));
  Serial.print(F("  Blink mode: "));       Serial.println((int)(buf[5]&0x03));
  
  Serial.print(F("Distance: "));           Serial.println(Slave2Host(buf[7], buf[6]));
  Serial.print(F("Force: "));              Serial.println(Slave2Host(buf[9], buf[8]));
  Serial.print(F("Shield: 0x"));           Serial.println(buf[10], HEX);
  Serial.print(F("  A: "));                Serial.println((buf[10]&(1<<0)?"contact":"0"));
  Serial.print(F("  B: "));                Serial.println((buf[10]&(1<<2)?"contact":"0"));
  Serial.print(F("  C: "));                Serial.println((buf[10]&(1<<1)?"contact":"0"));
  Serial.print(F("  D: "));                Serial.println((buf[10]&(1<<3)?"contact":"0"));
  Serial.print(F("  center: "));           Serial.println((buf[10]&(1<<4)?"contact":"0"));

  Serial.println(F("Color :"));
  Serial.print(F("  red: "));              Serial.println((int)buf[11]);
  Serial.print(F("  green: "));            Serial.println((int)buf[12]);
  Serial.print(F("  blue: "));             Serial.println((int)buf[13]);

  Serial.print(F("Id: "));                 Serial.println((int)buf[14]);
  Serial.print(F("Slave type: "));         Serial.println((int)buf[15]);
  Serial.print(F("firmware version: "));   Serial.println((int)buf[16]);
  Serial.print(F("Distance threshold: ")); Serial.println(Slave2Host(buf[18], buf[17]));
  Serial.print(F("Force threshold: "));    Serial.println(Slave2Host(buf[20], buf[19]));

  Serial.println(F("------------*------------"));
  return true;
}
//

//  Set/get functions
int SlaveController::setRegister(uint8_t ID, uint8_t reg, uint8_t regSize, uint8_t* val)
{
  const uint8_t messageLength = 3+regSize;
  uint8_t crc = ID + messageLength + INST_WRITE + reg;
  
  disableRX();
  printHeader(SLAVE_START, ID, messageLength);
  printf(INST_WRITE);
  printf(reg);
  for(uint8_t i=0; i<regSize; i++)
  {
    printf(val[regSize-1-i]);
    crc += val[regSize-1-i];
  }
  printf((~(crc))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return readBuf(6);
  return 0;
}
int SlaveController::getRegister(uint8_t ID, uint8_t reg, uint8_t regSize)
{
  const uint8_t messageLength = 4;

  disableRX();
  printHeader(SLAVE_START, ID,messageLength);
  printf(INST_READ);
  printf(reg);
  printf(regSize);
  printf((~(ID + messageLength + INST_READ + reg + regSize))&0xFF);
  flush();
  enableRX();

  const uint8_t responseLength = 6 + regSize;
  uint8_t buf[responseLength];
  if(readBuf(responseLength, buf) < responseLength) return -1;
  
  if(regSize == 2) return Slave2Host(buf[6], buf[5]);
  else if(regSize == 1) return buf[5];
  else return -2;
}
int SlaveController::setPermanentID(uint8_t ID, uint8_t newID)
{
  Network.setTimeout(10);
  uint8_t error = 0;
  for(;;)
  {
    error = (lockEeprom(ID, false)!= 6 && ID != BROADCAST_ID)?1:0;                            if(error) break;
    error = (setRegister(ID, REG_SIZE + EEPROM_ID, 1, &newID)!= 6 && ID != BROADCAST_ID)?2:0; if(error) break;
    error = (lockEeprom(newID, true)!= 6 && ID != BROADCAST_ID)?3:0;                          if(error) break;
    break;
  }
  Network.setTimeout(3);
  return error;
}
//










