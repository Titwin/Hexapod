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
uint8_t funnyControl = 0;
int distance1, distance2;
float sorient_x, orient_x;

uint8_t failMot[MAX_MOTOR_ON_CHANNEL];
int posMot[MAX_MOTOR_ON_CHANNEL];
int torqueMot[MAX_MOTOR_ON_CHANNEL];
int tempMot[MAX_MOTOR_ON_CHANNEL];
int goalPosMot[MAX_MOTOR_ON_CHANNEL];

uint8_t idsSend[MAX_MOTOR_ON_CHANNEL];
int posSend[MAX_MOTOR_ON_CHANNEL];

int analogIn[9];

SCS15Controller SCS15;
SlaveController SLAVE;
Watchdog WDOG(WDTO_60MS);
Network NET;
RPIController RPI;
AnalogScanner ANALOG;
//


//  Initialization
void setup()
{
  //  Debug
  pinMode(13,OUTPUT); digitalWrite(13, LOW);

  //  Turn power up and enable all legs
  pinMode(EN_5V,OUTPUT);    digitalWrite(EN_5V, HIGH);
  pinMode(EN_Front,OUTPUT); digitalWrite(EN_Front,HIGH);
  pinMode(EN_Back,OUTPUT);  digitalWrite(EN_Back, HIGH);
  pinMode(EN_Leg0,OUTPUT);  digitalWrite(EN_Leg0, HIGH);
  pinMode(EN_Leg1,OUTPUT);  digitalWrite(EN_Leg1, HIGH);
  pinMode(EN_Leg2,OUTPUT);  digitalWrite(EN_Leg2, HIGH);
  pinMode(EN_Leg3,OUTPUT);  digitalWrite(EN_Leg3, HIGH);
  pinMode(EN_Leg4,OUTPUT);  digitalWrite(EN_Leg4, HIGH);
  pinMode(EN_Leg5,OUTPUT);  digitalWrite(EN_Leg5, HIGH);
  
  //  Read if an unexpected shutdown occured and initialize network manager
  boardControl = EEPROM.read(0);
  if(NET.enable())
  {
    SCS15.initialize();
    RPI.initialize();
  
    // 0x01 = RPI_TARGET_CONTROL
    uint8_t dummy = CTRL_WDT_SHUTDOWN | boardControl;
    RPI.sendMsg(RPI_INST_READ | 0x01, 1, &dummy);
    
    frameRateTime = RPI_OFF_LOOP_TIME;
    NET.loadFromEeprom();
    NET.setLoopTime(frameRateTime);
  }
  else
  {
    SCS15.initialize();
    RPI.initialize();
  
    boardControl = CTRL_ANALOG_ENABLE | CTRL_SCS15_SCHEDULER_ENABLE;
    frameRateTime = RPI_ON_LOOP_TIME;
    NET.setup();
    NET.setLoopTime(frameRateTime);
  }

  //  Other
  ANALOG.initialize();

  //  Initialize and reset fault latch
  pinMode(FAULT_LATCH_SET,OUTPUT); digitalWrite(FAULT_LATCH_SET, HIGH);
  pinMode(FAULT_LATCH_RST,OUTPUT); digitalWrite(FAULT_LATCH_RST, LOW);  digitalWrite(FAULT_LATCH_RST, HIGH);

  //  Save default controle byte
  EEPROM.update(0, boardControl);
    
  //  Start watchdog
  WDOG.on();
}
//








//  Program
unsigned long start, RPIwdt;
void loop()
{
  //  START
  digitalWrite(13,LOW);
  WDOG.reset();
  start = millis();

  //SCS15.enableTorque(0xFE, true);
  //SCS15.setPosition(0xFE, 512);
  
  //  NETWORK
  /*  one frame is composed by several step or jobs.
      some of them are the same (fixed function), but others are defined with a diferent refresh rate
      these are performed only if the µC have enough time.
    
      A special behaviour is the node scan:
      each frame the network manager perform a ping on disconected node to test if it's operational again
      the number of timeout avalaible is defined as a retry integer gived in parameter of the fixedfunction
      default value is 1 : the network manager allow one timeout to be rechead each frame for nodescan or
      a retry connection
  */
  int retry = NET.fixedFunction();
  if(retry > 0) NET.nodeScan();
  if(  (boardControl & CTRL_SCS15_SCHEDULER_ENABLE) && !(boardControl & CTRL_RPI_DOWN))
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
  }
  else if(RPIwdt > 10000)
  {
    boardControl |= CTRL_RPI_DOWN;
    frameRateTime = RPI_OFF_LOOP_TIME;
    NET.setLoopTime(frameRateTime);
  }
  else if(RPIwdt > 3000)
    RPI.sendPing();

  //  Analog scanner update
  if(boardControl|CTRL_ANALOG_ENABLE)
      ANALOG.update(start + frameRateTime);


  //  END
  digitalWrite(13,HIGH);
  while(millis() - start < frameRateTime);
}
//

