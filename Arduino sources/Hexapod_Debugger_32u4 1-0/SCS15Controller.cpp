#include "SCS15Controller.h"

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_SYNC_WRITE 0x83

#define ACKNOWLEGE 0x06

//  Default
SCS15Controller::SCS15Controller(){}
//


//  Special
bool SCS15Controller::ping(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(SCS15_START, ID,messageLength);
  printf(INST_PING);
  printf((~(ID + messageLength + INST_PING))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == ACKNOWLEGE);
  return false;
}

bool SCS15Controller::reset(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(SCS15_START, ID,messageLength);
  printf(INST_RESET);
  printf((~(ID + messageLength + INST_RESET))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == ACKNOWLEGE);
  return false;
}


bool SCS15Controller::debug(uint8_t ID)
{
  const uint8_t messageLength = 4;
  const uint8_t readSize = 69;

  disableRX();
  printHeader(SCS15_START, ID,messageLength);
  printf(INST_READ);
  printf(0);
  printf(readSize);
  printf((~(ID + messageLength + INST_READ + readSize))&0xFF);
  flush();
  enableRX();

  const uint8_t responseLength = 6+readSize;
  uint8_t buf[responseLength];
  if(readBuf(responseLength, buf) < responseLength) return false;

  Serial.println(F("-------Motor print-------"));
  Serial.print(F("Model number: "));       Serial.print(buf[6]);       Serial.print('.');      Serial.println(buf[5]);
  Serial.print(F("Version: "));            Serial.print(buf[9]);       Serial.print('.');      Serial.println(buf[8]);
  Serial.print(F("ID: "));                 Serial.println((int)buf[10]);
  Serial.print(F("Baud rate: "));          Serial.println((int)buf[11]);
  Serial.print(F("Return delay time: "));  Serial.println((int)buf[12]);
  Serial.print(F("return level: "));       Serial.println((int)buf[13]);
  Serial.print(F("Angle limit min: "));    Serial.println(SCS2Host(buf[15], buf[14]));
  Serial.print(F("Angle limit max: "));    Serial.println(SCS2Host(buf[17], buf[16]));
  Serial.print(F("Temperature limit: "));  Serial.println((int)buf[18]);
  Serial.print(F("Voltage limit max: "));  Serial.println((int)buf[19]);
  Serial.print(F("Voltage limit min: "));  Serial.println((int)buf[20]);
  Serial.print(F("Torque max: "));         Serial.println(SCS2Host(buf[22], buf[21]));
  Serial.print(F("Alarm led: "));          Serial.println((buf[23]?"yes":"no"));
  Serial.print(F("Alarm shutdown: "));     Serial.println((int)buf[24]);
  Serial.print(F("Compliance P: "));       Serial.println((int)buf[26]);
  Serial.print(F("Compliance D: "));       Serial.println((int)buf[27]);
  Serial.print(F("Compliance I: "));       Serial.println((int)buf[28]);
  Serial.print(F("Punch: "));              Serial.println(SCS2Host(buf[30], buf[29]));
  Serial.print(F("Dead zone CW: "));       Serial.println((int)buf[31]);
  Serial.print(F("Dead zone CCW: "));      Serial.println((int)buf[32]);
  Serial.print(F("Current max: "));        Serial.println(SCS2Host(buf[34], buf[33]));
  Serial.print(F("Offset: "));             Serial.println(SCS2Host(buf[36], buf[35]));

  Serial.print(F("\nTorque enable: "));    Serial.println((buf[45]?"true":"false"));
  Serial.print(F("Led: "));                Serial.println((buf[46]?"yes":"no"));
  Serial.print(F("Goal position: "));      Serial.println(SCS2Host(buf[48], buf[47]));
  Serial.print(F("Goal time: "));          Serial.println(SCS2Host(buf[50], buf[49]));
  Serial.print(F("Goal speed: "));         Serial.println(SCS2Host(buf[52], buf[51]));
  Serial.print(F("EEPROM: "));             Serial.println((buf[53]?"locked":"unlocked"));

  Serial.print(F("\nPosition: "));         Serial.println(SCS2Host(buf[62], buf[61]));
  Serial.print(F("Speed: "));              Serial.println(SCS2Host(buf[64], buf[63]));
  Serial.print(F("Load: "));               Serial.println(SCS2Host(buf[66], buf[65]));
  Serial.print(F("Voltage: "));            Serial.println((int)buf[67]);
  Serial.print(F("Temperature: "));        Serial.println((int)buf[68]);
  
  //Serial.print("Registred instruction: ");Serial.println((int)buf[69]);
  //Serial.print("Error: ");              Serial.println((int)buf[5]);
  //Serial.print("Moving: ");             Serial.println((buf[71]?"yes":"no"));
  //Serial.print("Vir position: ");       Serial.println(SCS2Host(buf[73], buf[72]));
  //Serial.print("Current: ");            Serial.println(SCS2Host(buf[75], buf[74]));

  Serial.println(F("------------*------------"));
  return true;
}
//


//  private functions
int SCS15Controller::readBuf(uint8_t len, uint8_t *buf)
{
  if(buf) return scanf(buf,len);
  else
  {
    uint8_t buftmp[10];
    return scanf(buftmp,len);
  }
}

int SCS15Controller::SCS2Host(uint8_t DataL, uint8_t DataH)
{
  return ((int)DataH<<8)|DataL;
}

void SCS15Controller::host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
  *DataL = (Data>>8);
  *DataH = (Data&0xFF);
}
//


//  Set/get functions
int SCS15Controller::setRegister(uint8_t ID,uint8_t reg,uint8_t regSize,uint8_t* val)
{
  const uint8_t messageLength = 3+regSize;
  uint8_t crc = ID + messageLength + INST_WRITE + reg;
  
  disableRX();
  printHeader(SCS15_START, ID,messageLength);
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

int SCS15Controller::getRegister(uint8_t ID,uint8_t reg,uint8_t regSize)
{
  const uint8_t messageLength = 4;

  disableRX();
  printHeader(SCS15_START, ID,messageLength);
  printf(INST_READ);
  printf(reg);
  printf(regSize);
  printf((~(ID + messageLength + INST_READ + reg + regSize))&0xFF);
  flush();
  enableRX();

  const uint8_t responseLength = 6 + regSize;
  uint8_t buf[responseLength];
  if(readBuf(responseLength, buf) < responseLength) return -1;
  
  if(regSize == 2) return SCS2Host(buf[6], buf[5]);
  else if(regSize == 1) return buf[5];
  else return -2;
}

void SCS15Controller::syncSetRegister(uint8_t IDN, uint8_t* ID,uint8_t regStart, uint8_t regSize, uint8_t* allRegisterValue)
{
  const uint8_t messageLength = (regSize+1)*IDN+4;
  uint8_t crc = BROADCAST_ID + messageLength + INST_SYNC_WRITE + regStart + regSize;
  
  disableRX();
  printHeader(SCS15_START, BROADCAST_ID, messageLength);
  printf(INST_SYNC_WRITE);
  printf(regStart);
  printf(regSize);
  
  for(uint8_t i=0; i<IDN; i++)
  {
    printf(ID[i]);
    crc += ID[i];
    
    for(uint8_t j=0; j<regSize; j++)
    {
      printf(allRegisterValue[i*regSize-1-j]);
      crc += allRegisterValue[i*regSize-1-j];
    }
  }
  printf((~crc)&0xFF);
  flush();
  enableRX();
}

int SCS15Controller::setPermanentID(uint8_t ID, uint8_t newID)
{
  Network.setTimeout(20);
  uint8_t error = 0;
  for(;;)
  {
    error = (lockEeprom(ID, false)!= 6 && ID != BROADCAST_ID)?1:0;     if(error) break;
    error = (setTemporaryID(ID, newID)!= 6 && ID != BROADCAST_ID)?2:0; if(error) break;
    error = (lockEeprom(newID, true)!= 6 && ID != BROADCAST_ID)?3:0;      if(error) break;
    break;
  }
  Network.setTimeout(3);
  return error;
}
//

