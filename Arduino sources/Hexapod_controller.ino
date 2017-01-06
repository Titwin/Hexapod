//  File inclusion & macro define

#include "Configuration.h"

#include "Arduino.h"
#include "Network.h"
#include "Watchdog.h"
#include "RPIController.h"
#include "AnalogScanner.h"
//

//  Global variable initialization
unsigned long frameRateTime = RPI_ON_LOOP_TIME;
uint8_t boardControl;
uint8_t failMot[MAX_MOTOR_ON_CHANNEL];

int posMot[MAX_MOTOR_ON_CHANNEL];
int torqueMot[MAX_MOTOR_ON_CHANNEL];
int tempMot[MAX_MOTOR_ON_CHANNEL];
int goalPosMot[MAX_MOTOR_ON_CHANNEL];

uint8_t idsSend[MAX_MOTOR_ON_CHANNEL];
int posSend[MAX_MOTOR_ON_CHANNEL];

uint16_t analogIn[MAX_ANALOG_INPUT];

SCS15Controller SCS15;
Watchdog WDOG(WDTO_60MS);
Network NET;
RPIController RPI;
AnalogScanner ANALOG;
//


//  Initialization
void setup()
{
  //  debug pin init
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);

  //  initialize PWM output
  for(uint8_t i=0; i<MAX_ANALOG_OUTPUT; i++)
  {
    //  5 and 6 analog out not available due to interaction with millis. See arduino analogWrite ref for more info
    if(FIRST_ANALOG_OUTPUT + i == 5) continue;
    else if(FIRST_ANALOG_OUTPUT + i == 6) continue;
    pinMode(FIRST_ANALOG_OUTPUT + i, OUTPUT);
  }

  //  Initialize controllers
  SCS15.initialize();
  RPI.initialize();

  //  Read if an unexpected shutdown occured and initialize network manager
  boardControl = EEPROM.read(0);
  if(boardControl & CTRL_WDT_SHUTDOWN)
  {
    // 0x01 = RPI_TARGET_CONTROL
    RPI.sendMsg(RPI_INST_READ | 0x01, 1, &boardControl); 
    
    boardControl &= ~CTRL_WDT_SHUTDOWN;
    frameRateTime = RPI_OFF_LOOP_TIME;
    NET.loadFromEeprom();
    NET.setLoopTime(frameRateTime);
  }
  else
  {
    boardControl = CTRL_ANALOG_ENABLE | CTRL_SCS15_SCHEDULER_ENABLE;
    frameRateTime = RPI_ON_LOOP_TIME;
    NET.setup();
    NET.setLoopTime(frameRateTime);
  }
  
  //  Save default controle byte
  EEPROM.update(0, boardControl);

  //  Start analog scanner
  if(boardControl & CTRL_ANALOG_ENABLE)
    ANALOG.initialize();
    
  //  Start watchdog
  //WDOG.on();
  digitalWrite(13,HIGH);
}
//








//  Program
unsigned long start, microstart, RPIwdt;
void loop()
{
  //  START
  digitalWrite(13,LOW);
  WDOG.reset();
  start = millis();

  //  NETWORK
  /*  one frame is composed by several step or jobs.
      some of them are the same (fixed function), but others are defined with a diferent refresh rate
      these are performed only if the µC have enough time.
    
      A special behaviour is the node scan:
      each frame the network manager perform a ping on disconected node to test if it's operational again
      the number of timeout avalible is defined as a retry integer gived in parameter of the fixedfunction
      default value is 1 : the network manager allow one timeout to be rechead each frame for nodescan or
      a retry connection
  */
  int retry = NET.fixedFunction();
  if(retry > 0) NET.nodeScan();
  if(  (boardControl & CTRL_SCS15_SCHEDULER_ENABLE) &&
      !(boardControl & CTRL_RPI_DOWN))
  {
    while(millis() - start < NETWORK_TIME)
      NET.taskScheduled();
  }

  
  
  //  RPI COMMUNICATION
  RPI.update(start + frameRateTime - 1);
  
  RPIwdt = millis() - RPI.lastValidMsgTimestamp;
  if(boardControl & CTRL_RPI_DOWN)
  {
    RPI.sendPing();
    if(millis() > 5*frameRateTime && RPIwdt < 5*frameRateTime)
    {
      boardControl &= ~CTRL_RPI_DOWN;
      frameRateTime = RPI_ON_LOOP_TIME;
      NET.setLoopTime(frameRateTime);
    }
    
    //  alternative code if RPI down
  }
  else if(RPIwdt > 10000)
  {
    boardControl |= CTRL_RPI_DOWN;
    frameRateTime = RPI_OFF_LOOP_TIME;
    NET.setLoopTime(frameRateTime);
  }
  else if(RPIwdt > 3000)
    RPI.sendPing();


  
  //  ANALOG INPUT
  if(boardControl & CTRL_ANALOG_ENABLE)
    ANALOG.update(start + frameRateTime - 1);
  


  //  END
  digitalWrite(13,HIGH);
  while(millis() - start < frameRateTime);
}
//

