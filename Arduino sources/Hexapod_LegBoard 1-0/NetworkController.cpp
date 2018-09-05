//  File inclusion & macro define
#include "NetworkController.h"
#include <avr/wdt.h>

#define serial Serial1
#define enableRX()  {UCSR1B |= (1<<RXEN1);  UCSR1B &=~(1<<TXEN1);  DDRD &= ~(1<<DDD2); PORTD |= (1<<PORTD2);}
#define disableRX() {UCSR1B &=~(1<<RXEN1);  UCSR1B |= (1<<TXEN1);}
#define flush() (serial.flush())



//  default
NetworkController::NetworkController(){}
//

//  special
void NetworkController::initialize(uint8_t startingId)
{
  id = startingId;
  lastValidCharTimestamp = millis();
  
  serial.begin(1000000);
  serial.setTimeout(3);
  enableRX();

  callbackArray[0] = NULL;  //  NC_INST_PING
  callbackArray[1] = NULL;  //  NC_INST_ACTION
  callbackArray[2] = NULL;  //  NC_INST_READ
  callbackArray[3] = NULL;  //  NC_INST_WRITE
  callbackArray[4] = NULL;  //  NC_INST_ACTION
  callbackArray[5] = NULL;  //  NC_INST_RESET
}
//


//  network managment
void NetworkController::update()
{
  while(serial.available())
  {
    int c = serial.read();
    if(c >= 0) parseChar(c);
  }
}
void NetworkController::resetState()
{
  state = 0;
  msgIndex = 0;
  msgLength = 0;
  msgInst = 0;
  msgId = 0;
  crc = 0;
}
void inline NetworkController::printHeader(const uint8_t& id, const uint8_t& msgSize, const uint8_t& start)
{
  serial.write(start);
  serial.write(start);
  serial.write(id);
  serial.write(msgSize);
}
void NetworkController::parseChar(uint8_t c)
{
  lastValidCharTimestamp = millis();
  switch(state)
  {
    // parsing msg header
    case 0: case 1:
      if(c == NC_START_BYTE) state++;
      else resetState();
      break;

    case 2:
      if(c == id || c == NC_BROADCAST_ID) state++;
      else resetState();
      crc = c;
      msgId = c;
      break;

    case 3:
      crc += c;
      msgLength = c;
      state++;
      break;
      
    case 4:
      crc += c;
      msgInst = c;
      if(msgInst == NC_INST_PING || msgInst == NC_INST_ACTION || msgInst == NC_INST_RESET)
        state = 6;
      else if(msgInst == NC_INST_REPLY) // ignore echo & others messages
        resetState();
      else state++;
      break;

    // save incomming message
    case 5:
      crc += c;
      currentMsg[msgIndex] = c;
      msgIndex++;
      if(msgIndex+2 >= msgLength)
        state = 6;
      break;

    // check crc and respond
    case 6:
      if(c == ((~crc)&0xFF))
      {
        switch(msgInst)
        {
          case NC_INST_PING:
            if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
            if(callbackArray[0])(*callbackArray[0])(0, NULL);
            break;

          case NC_INST_ACTION:
            if(callbackArray[1]) (*callbackArray[1])(0, NULL);
            break;
            
          case NC_INST_READ:
            if(msgId == id && callbackArray[2]) (*callbackArray[2])(msgLength-2, currentMsg);
            break;
            
          case NC_INST_WRITE:
            if(callbackArray[3]) (*callbackArray[3])(msgLength-2, currentMsg);
            break;
            
          case NC_INST_REG_WRITE:
            if(callbackArray[4]) (*callbackArray[4])(msgLength-2, currentMsg);
            if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
            break;

          case NC_INST_RESET:
            if(callbackArray[5])(*callbackArray[5])(0, NULL);
            if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
            wdt_enable(WDTO_15MS);
            while(1);
            break;
            
          default: break;
        }
      }
      resetState();
      break;
      
    default:
      resetState();
      break;
  }
}
//

//  public functions
bool NetworkController::assignCallback(uint8_t instructionType, NetworkCallback callback)
{
  switch(instructionType)
  {
    case NC_INST_PING:        callbackArray[0] = callback; break;
    case NC_INST_ACTION:      callbackArray[1] = callback; break;
    case NC_INST_READ:        callbackArray[2] = callback; break;
    case NC_INST_WRITE:       callbackArray[3] = callback; break;
    case NC_INST_REG_WRITE:   callbackArray[4] = callback; break;
    case NC_INST_RESET:       callbackArray[5] = callback; break;
    default: return false;
  }
  return true;
}
void NetworkController::sendRegister(const uint8_t& instruction, const uint8_t& regSize, const uint8_t* reg)
{
  const uint8_t messageLength = 2 + regSize;
  uint8_t crc = id + messageLength + instruction;

  disableRX()
  printHeader(id, messageLength);
  serial.write(instruction);
  for(uint8_t i=0; i<regSize; i++)
  {
    serial.write(reg[i]);
    crc += reg[i];
  }
  serial.write((~(crc))&0xFF);
  flush();
  enableRX()
}
unsigned long NetworkController::getLastTimestamp(){return lastValidCharTimestamp;}
bool NetworkController::isPersonalId(){return msgId == id;};
void NetworkController::setId(const uint8_t& newid){id = newid;}
//

