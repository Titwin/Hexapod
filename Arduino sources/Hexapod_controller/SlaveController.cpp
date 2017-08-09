#include "SlaveController.h"
  
#define slavePort Serial
#define enableRX()  {UCSR0B |= (1<<RXEN0);  UCSR0B &=~(1<<TXEN0);  DDRE &= ~(1<<DDE1); PORTE |= (1<<PORTE1);}
#define disableRX() {UCSR0B &=~(1<<RXEN0);  UCSR0B |= (1<<TXEN0);}

#define printf(args) (slavePort.write(args))
#define flush() (slavePort.flush())
#define scanf(buf,len) (slavePort.readBytes(buf,len))
#define printHeader(id,msgSize) {printf(Slave_startByte); printf(Slave_startByte); printf(id); printf(msgSize);}



//  Default
SlaveController::SlaveController(){}
//


//  Special
bool SlaveController::ping(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(ID, messageLength);
  printf(INST_PING);
  printf((~(ID + messageLength + INST_PING))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return readBuf(6);
  return 0;
}


bool SlaveController::action(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(ID, messageLength);
  printf(INST_ACTION);
  printf((~(ID + messageLength + INST_ACTION))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return readBuf(6);
  return 0;
}

bool SlaveController::reset(uint8_t ID)
{
  const uint8_t messageLength = 2;
  
  disableRX();
  printHeader(ID, messageLength);
  printf(INST_RESET);
  printf((~(ID + messageLength + INST_RESET))&0xFF);
  flush();
  enableRX();

  if(ID != BROADCAST_ID) return readBuf(6);
  return 0;
}


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

int SlaveController::SCS2Host(uint8_t DataL, uint8_t DataH)
{
  return ((int)DataH<<8)|DataL;
}

float SlaveController::bytes2Float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  uint8_t ptr[4] = {b0,b1,b2,b3};
  return *(float *)&ptr;;
}



void SlaveController::host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
  *DataL = (Data>>8);
  *DataH = (Data&0xFF);
}
//


//  Set/get functions
int SlaveController::setRegister(uint8_t ID,uint8_t reg,uint8_t regSize,uint8_t* val)
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

int SlaveController::getRegister(uint8_t ID, uint8_t reg, uint8_t regSize)
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

float SlaveController::getFloatRegister(uint8_t ID, uint8_t reg)
{
  const uint8_t messageLength = 4;

  disableRX();
  printHeader(ID,messageLength);
  printf(INST_READ);
  printf(reg);
  printf(4);
  printf((~(ID + messageLength + INST_READ + reg + 4))&0xFF);
  flush();
  enableRX();

  uint8_t buf[10];
  if(readBuf(10, buf) < 10) return -4.2e12;
  return bytes2Float(buf[8],buf[7],buf[6],buf[5]);
}


