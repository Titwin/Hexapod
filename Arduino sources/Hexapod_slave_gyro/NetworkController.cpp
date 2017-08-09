//  File inclusion & macro define
#include "NetworkController.h"

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_RESET 0x06
#define INST_SYNC_WRITE 0x83

#define enableRX()  {UCSR0B |= (1<<RXEN0);  UCSR0B &=~(1<<TXEN0);  DDRD &= ~(1<<DDD0); PORTD |= (1<<PORTD0);}
#define disableRX() {UCSR0B &=~(1<<RXEN0);  UCSR0B |= (1<<TXEN0);}
//

//  extern variable link
extern float speed_x;
extern float orient_x;
extern uint16_t samples;


float placeHolder = 5012.f;
//

//  default
NetworkController::NetworkController(){}
//


//  special
void NetworkController::initialize()
{
  uint8_t storeId = EEPROM.read(S_ID);
  if(storeId) id = storeId;
  else id = DEFAULT_ID;
  
  Serial.begin(1000000);
  Serial.setTimeout(3);
  enableRX();
}
//


//  network managment
void NetworkController::resetState()
{
  state = 0;
  msgIndex = 0;
  msgLength = 0;
  msgInst = 0;
  msgId = 0;
  crc = 0;
}

void NetworkController::parseChar(uint8_t c)
{
  lastValidMsgTimestamp = millis();
  switch(state)
  {
    // parsing msg header
    case 0:
      if(c == startByte) state++;
      else resetState();
      break;
      
    case 1:
      if(c == startByte) state++;
      else resetState();
      break;

    case 2:
      crc = c;
      msgId = c;
      if(msgId == id || msgId == BROADCAST_ID) state++;
      else resetState();
      break;

    case 3:
      crc += c;
      msgLength = c;
      state++;
      break;
      
    case 4:
      crc += c;
      msgInst = c;
      if(msgInst == INST_PING) state = 6;
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
          case INST_PING:
            if(msgId == id)
            {
              disableRX();
              printHeader(id, 2);
              Serial.write(INST_PING);
              Serial.write((~(id + 2 + INST_PING))&0xFF);
              Serial.flush();
              enableRX();
            }
            break;
            
          case INST_READ:
            if(msgId == id)
            {
              switch(currentMsg[0])
              {
                case S_SPEED_ORIENTATION:
                  sendRegister(4, (uint8_t*)&speed_x);
                  break;
                case S_ORIENTATION:
                  sendRegister(4, (uint8_t*)&orient_x);
                  break;
                default: break;
              }
            }
            break;
            
          case INST_RESET:
            samples = 0;
            break;
            
          case INST_WRITE:
            if(currentMsg[0] == S_ID && msgId != BROADCAST_ID) id = currentMsg[1];
            break;
            
          case INST_REG_WRITE:
            if(currentMsg[0] == S_ID && msgId != BROADCAST_ID)
            {
              id = currentMsg[1];
              EEPROM.update(S_ID, id);
            }
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

//  Set/get functions
void NetworkController::sendRegister(uint8_t regSize,uint8_t* reg)
{
  const uint8_t messageLength = 2 + regSize;
  uint8_t crc = id + messageLength + INST_READ;
  
  disableRX();
  printHeader(id, messageLength);
  Serial.write(INST_READ);
  for(uint8_t i=0; i<regSize; i++)
  {
    Serial.write(reg[regSize-1-i]);
    crc += reg[regSize-1-i];
  }
  Serial.write((~(crc))&0xFF);
  Serial.flush();
  enableRX();
}
//

