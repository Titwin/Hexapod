#include "SCS15Controller.h"

#ifndef SCS15_CHANEL
  #define SCS15_CHANEL 2
#endif

#if SCS15_CHANEL == 0
  #ifdef VERBOSE
    #pragma message("SCS15 Controller mapped by preprocessor on USART 0")
  #endif

  #define scs15Port Serial
  #define enableRX()  {UCSR0B |= (1<<RXEN0);  UCSR0B &=~(1<<TXEN0);  DDRE &= ~(1<<DDE1); PORTE |= (1<<PORTE1);}
  #define disableRX() {UCSR0B &=~(1<<RXEN0);  UCSR0B |= (1<<TXEN0);}
  
#elif SCS15_CHANEL == 1
  #ifdef VERBOSE
    #pragma message("SCS15 Controller mapped by preprocessor on USART 1")
  #endif

  #define scs15Port Serial1
  #define enableRX()  {UCSR1B |= (1<<RXEN2);  UCSR1B &=~(1<<TXEN1);  DDRD &= ~(1<<DDD3); PORTD |= (1<<PORTD3);}
  #define disableRX() {UCSR1B &=~(1<<RXEN2);  UCSR1B |= (1<<TXEN1);}
  
#elif SCS15_CHANEL == 2
  #ifdef VERBOSE
    #pragma message("SCS15 Controller mapped by preprocessor on USART 2")
  #endif

  #define scs15Port Serial2
  #define enableRX()  {UCSR2B |= (1<<RXEN2);  UCSR2B &=~(1<<TXEN2);  DDRH &= ~(1<<DDH1); PORTH |= (1<<PORTH1);}
  #define disableRX() {UCSR2B &=~(1<<RXEN2);  UCSR2B |= (1<<TXEN2);}

#elif SCS15_CHANEL == 3
  #ifdef VERBOSE
    #pragma message("SCS15 Controller mapped by preprocessor on USART 3")
  #endif

  #define scs15Port Serial3
  #define enableRX()  {UCSR3B |= (1<<RXEN3);  UCSR3B &=~(1<<TXEN3);  DDRJ &= ~(1<<DDJ1); PORTJ |= (1<<PORTJ1);}

#else
  #ifdef VERBOSE
    #pragma message("Incorrect SCS15 chanel selected !!!")
    #pragma message("SCS15 Controller mapped by preprocessor on USART 2")
  #endif

  #define scs15Port Serial2
  #define enableRX()  {UCSR2B |= (1<<RXEN2);  UCSR2B &=~(1<<TXEN2);  DDRH &= ~(1<<DDH1); PORTH |= (1<<PORTH1);}
  #define disableRX() {UCSR2B &=~(1<<RXEN2);  UCSR2B |= (1<<TXEN2);}
#endif

#define printf(args) (scs15Port.write(args))
#define flush() (scs15Port.flush())
#define scanf(buf,len) (scs15Port.readBytes(buf,len))
#define printHeader(id,msgSize) {printf(startByte); printf(startByte); printf(id); printf(msgSize);}



//  Default
SCS15Controller::SCS15Controller(){}
//


//  Special
void SCS15Controller::initialize()
{
  scs15Port.begin(1000000);
  scs15Port.setTimeout(3);
  enableRX();
}

bool SCS15Controller::ping(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(ID,messageLength);
  printf(INST_PING);
  printf((~(ID + messageLength + INST_PING))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return (readBuf(6) == 6);
  return false;
}

void SCS15Controller::debug(uint8_t ID)
{
  const uint8_t messageLength = 4;
  const uint8_t readSize = 69;

  disableRX();
  printHeader(ID,messageLength);
  printf(INST_READ);
  printf(0);
  printf(readSize);
  printf((~(ID + messageLength + INST_READ + readSize))&0xFF);
  flush();
  enableRX();

  const uint8_t responseLength = 6+readSize;
  uint8_t buf[responseLength];
  if(readBuf(responseLength, buf) < responseLength) return;

  Serial.println("-------Motor print-------");
  Serial.print("Model number: ");       Serial.print(buf[6]);       Serial.print('.');      Serial.println(buf[5]);
  Serial.print("Version: ");            Serial.print(buf[9]);       Serial.print('.');      Serial.println(buf[8]);
  Serial.print("ID: ");                 Serial.println((int)buf[10]);
  Serial.print("Baud rate: ");          Serial.println((int)buf[11]);
  Serial.print("Return delay time: ");  Serial.println((int)buf[12]);
  Serial.print("return level: ");       Serial.println((int)buf[13]);
  Serial.print("Angle limit min: ");    Serial.println(SCS2Host(buf[15], buf[14]));
  Serial.print("Angle limit max: ");    Serial.println(SCS2Host(buf[17], buf[16]));
  Serial.print("Temperature limit: ");  Serial.println((int)buf[18]);
  Serial.print("Voltage limit max: ");  Serial.println((int)buf[19]);
  Serial.print("Voltage limit min: ");  Serial.println((int)buf[20]);
  Serial.print("Torque max: ");         Serial.println(SCS2Host(buf[22], buf[21]));
  Serial.print("Alarm led: ");          Serial.println((buf[23]?"yes":"no"));
  Serial.print("Alarm shutdown: ");     Serial.println((int)buf[24]);
  Serial.print("Compliance P: ");       Serial.println((int)buf[26]);
  Serial.print("Compliance D: ");       Serial.println((int)buf[27]);
  Serial.print("Compliance I: ");       Serial.println((int)buf[28]);
  Serial.print("Punch: ");              Serial.println(SCS2Host(buf[30], buf[29]));
  Serial.print("Dead zone CW: ");       Serial.println((int)buf[31]);
  Serial.print("Dead zone CCW: ");      Serial.println((int)buf[32]);
  Serial.print("Current max: ");        Serial.println(SCS2Host(buf[34], buf[33]));
  Serial.print("Offset: ");             Serial.println(SCS2Host(buf[36], buf[35]));

  Serial.print("\nTorque enable: ");    Serial.println((buf[45]?"true":"false"));
  Serial.print("Led: ");                Serial.println((buf[46]?"yes":"no"));
  Serial.print("Goal position: ");      Serial.println(SCS2Host(buf[48], buf[47]));
  Serial.print("Goal time: ");          Serial.println(SCS2Host(buf[50], buf[49]));
  Serial.print("Goal speed: ");         Serial.println(SCS2Host(buf[52], buf[51]));
  Serial.print("EEPROM: ");             Serial.println((buf[53]?"locked":"unlocked"));

  Serial.print("\nPosition: ");         Serial.println(SCS2Host(buf[62], buf[61]));
  Serial.print("Speed: ");              Serial.println(SCS2Host(buf[64], buf[63]));
  Serial.print("Load: ");               Serial.println(SCS2Host(buf[66], buf[65]));
  Serial.print("Voltage: ");            Serial.println((int)buf[67]);
  Serial.print("Temperature: ");        Serial.println((int)buf[68]);
  
  //Serial.print("Registred instruction: ");Serial.println((int)buf[69]);
  //Serial.print("Error: ");              Serial.println((int)buf[5]);
  //Serial.print("Moving: ");             Serial.println((buf[71]?"yes":"no"));
  //Serial.print("Vir position: ");       Serial.println(SCS2Host(buf[73], buf[72]));
  //Serial.print("Current: ");            Serial.println(SCS2Host(buf[75], buf[74]));

  Serial.println("------------*------------");
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
  printHeader(ID,messageLength);
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
  printHeader(ID,messageLength);
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

void SCS15Controller::syncSetRegister(uint8_t IDN, uint8_t* ID,uint8_t regStart, uint8_t singleMsgSize, uint8_t* allRegisterValue)
{
  const uint8_t messageLength = (singleMsgSize+1)*IDN+4;
  uint8_t crc = 0xfe + messageLength + INST_SYNC_WRITE + regStart + singleMsgSize;
  
  disableRX();
  printHeader(BROADCAST_ID, messageLength);
  printf(INST_SYNC_WRITE);
  printf(regStart);
  printf(singleMsgSize);
  
  for(uint8_t i=0; i<IDN; i++)
  {
    printf(ID[i]);
    crc += ID[i];
    
    for(uint8_t j=0; j<singleMsgSize; j++)
    {
      printf(allRegisterValue[i*singleMsgSize + j]);
      crc += allRegisterValue[i*singleMsgSize + j];
    }
  }
  printf((~crc)&0xFF);
  flush();
  enableRX();
}
//

