#include "RPIController.h"
#include "AnalogScanner.h"

//  target
#define RPI_TARGET_SHIFT  0
#define RPI_TARGET_MASK   (0x0F<<RPI_TARGET_SHIFT)

#define RPI_TARGET_CONTROL (1<<RPI_TARGET_SHIFT)
#define RPI_TARGET_SCS15   (2<<RPI_TARGET_SHIFT)
#define RPI_TARGET_ANALOG  (3<<RPI_TARGET_SHIFT)
//

//  register
#define RPI_POSITION 1
#define RPI_TORQUE 2
#define RPI_TEMPERATURE 3
#define RPI_FAIL_NODE 4
//


#ifndef RPI_CHANEL
  #define RPI_CHANEL 1
#endif

#if RPI_CHANEL == 0
  #ifdef VERBOSE
    #pragma message("RPI Controller mapped by preprocessor on USART 0")
  #endif

  #define rpiPort Serial

#elif RPI_CHANEL == 1
  #ifdef VERBOSE
    #pragma message("RPI Controller mapped by preprocessor on USART 1")
  #endif
          
  #define rpiPort Serial1

#elif RPI_CHANEL == 2
  #ifdef VERBOSE
    #pragma message("RPI Controller mapped by preprocessor on USART 2")
  #endif
  
  #define rpiPort Serial2

#elif RPI_CHANEL == 3
  #ifdef VERBOSE
    #pragma message("RPI Controller mapped by preprocessor on USART 3")
  #endif
          
  #define rpiPort Serial3

#else
  #ifdef VERBOSE
    #pragma message("Incorrect rpi chanel selected !!!")
    #pragma message("RPI Controller mapped by preprocessor on USART 1")
  #endif

  #define rpiPort Serial1

#endif



RPIController::RPIController(){}

void RPIController::initialize()
{
  rpiPort.begin(115200);
  rpiPort.setTimeout(1);
  resetState();
}

void RPIController::update(unsigned long stoptime)
{
  while(rpiPort.available() && millis()  < stoptime)
  {
    int c = rpiPort.read();
    if(c >= 0) parseChar(c);
  }
}

void RPIController::resetState()
{
  state = 0;
  msgIndex = 0;
  msgLength = 0;
  msgCode = 0;
  crc = 0;
}

void RPIController::parseChar(uint8_t c)
{
  switch(state)
  {
    case 0:
      if(c == startByte) state = 1;
      else resetState();
      break;
      
    case 1:
      if(c == startByte)state = 2;
      else resetState();
      break;

    case 2:
      msgCode = c;
      crc = msgCode;

      switch(msgCode & RPI_INST_MASK)
      {
        case RPI_INST_PING:
        case RPI_INST_ACK:
          state = 5;
          break;

        case RPI_INST_READ:
          switch(msgCode & RPI_TARGET_MASK)
          {
            case RPI_TARGET_CONTROL:
            case RPI_TARGET_ANALOG:
              state = 5;
              break;
              
            default:
              state = 3;
              break;
          }
          break;
          
        case RPI_INST_WRITE:
          state = 3;
          break;

        default:
          resetState();
          break;
      }
      break;

    case 3:
      crc += c;
      msgLength = c;
      state = 4;
      break;

    case 4:
      crc += c;
      currentMsg[msgIndex] = c;
      msgIndex++;
      if(msgIndex == msgLength)
        state = 5;
      break;

    case 5:
      if(c == crc)
      {
        lastValidMsgTimestamp = millis();
        switch(msgCode & RPI_INST_MASK)
        {
          case RPI_INST_PING:
            sendAck();
            break;

          case RPI_INST_ACK:
          case RPI_INST_READ:
          case RPI_INST_WRITE:
            processMsg();
            break;
            
          default:
            resetState();
            break;
        }
      }
      resetState();
      break;
      
    default:
      resetState();
      break;
  }
}

void RPIController::sendMsg(uint8_t code, uint8_t dataLength, uint8_t* data, uint8_t target2)
{
  if(rpiPort.availableForWrite() < dataLength+5) return;
  rpiPort.write(startByte);
  rpiPort.write(startByte);
  uint8_t crc = 0;
  rpiPort.write(code); crc += code;
  
  if(target2)
  {
    rpiPort.write(dataLength+1);
    rpiPort.write(target2);
    crc += dataLength + 1 + target2;
  }
  else if(dataLength)
  {
    rpiPort.write(dataLength);
    crc += dataLength;
  }
  
  for(uint8_t i=0; i<dataLength; i++)
  {
    crc += data[i];
    rpiPort.write(data[i]);
  }
  rpiPort.write(crc);
}





extern uint8_t boardControl;
extern uint8_t failMot[];

extern int posMot[];
extern int torqueMot[];
extern int tempMot[];
extern int goalPosMot[];

extern uint16_t analogIn[];
extern AnalogScanner ANALOG;



void RPIController::processMsg()
{
  switch(msgCode & RPI_TARGET_MASK)
  {
    case RPI_TARGET_CONTROL:
      if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
        sendMsg(RPI_INST_READ | RPI_TARGET_CONTROL, 1, &boardControl);
      else if((msgCode & RPI_INST_MASK) == RPI_INST_WRITE)
        boardControl = currentMsg[0];
      break;

    case RPI_TARGET_SCS15:
      switch(currentMsg[0])
      {
        case RPI_FAIL_NODE:
          if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
            sendMsg(RPI_INST_READ | RPI_TARGET_SCS15, MAX_MOTOR_ON_CHANNEL, (uint8_t*)failMot, RPI_FAIL_NODE);
          break;
          
        case RPI_POSITION:
          if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
            sendMsg(RPI_INST_READ | RPI_TARGET_SCS15, 2*MAX_MOTOR_ON_CHANNEL, (uint8_t*)posMot, RPI_POSITION);
          else if((msgCode & RPI_INST_MASK) == RPI_INST_WRITE)
          {
            for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++)
              goalPosMot[i] = (currentMsg[2*i+1] << 8) + (currentMsg[2*i+2]);
          }
          break;
          
        case RPI_TORQUE:
          if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
            sendMsg(RPI_INST_READ | RPI_TARGET_SCS15, 2*MAX_MOTOR_ON_CHANNEL, (uint8_t*)torqueMot, RPI_TORQUE);
          break;
          
        case RPI_TEMPERATURE:
          if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
            sendMsg(RPI_INST_READ | RPI_TARGET_SCS15, 2*MAX_MOTOR_ON_CHANNEL, (uint8_t*)tempMot, RPI_TEMPERATURE);
          break;
          
        default: break;
      }
      break;

    case RPI_TARGET_ANALOG:
      if((msgCode & RPI_INST_MASK) == RPI_INST_READ)
        sendMsg(RPI_INST_READ | RPI_TARGET_ANALOG, 2*MAX_ANALOG_INPUT, (uint8_t*)analogIn);
      else if((msgCode & RPI_INST_MASK) == RPI_INST_WRITE)
      {
        for(uint8_t i=0; i<MAX_ANALOG_OUTPUT; i++)
        {
          //  5 and 6 analog out not available due to interaction with millis. See arduino analogWrite ref for more info
          if(FIRST_ANALOG_OUTPUT + i == 5) continue;
          else if(FIRST_ANALOG_OUTPUT + i == 6) continue;
          
          analogWrite(FIRST_ANALOG_OUTPUT + i, currentMsg[i]);
        }
      }
      break;
      
    default: break;
  }
}

