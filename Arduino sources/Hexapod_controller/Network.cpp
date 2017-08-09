#include "Network.h"


extern SCS15Controller SCS15;
extern SlaveController SLAVE;

extern uint8_t funnyControl;
extern int distance1;
extern int distance2;
extern float sorient_x;
extern float orient_x;

extern uint8_t failMot[];
extern int posMot[];
extern int torqueMot[];
extern int tempMot[];
extern int goalPosMot[];

extern uint8_t idsSend[];
extern int posSend[];


Network::Network()
{
  loopTime = RPI_ON_LOOP_TIME;

  // relative to contact fail
  respondingIDN = 0;

  // relative to torque
  torque = false;

  // relative to discovery scan
  discoveryIndex = 0;
  specialScan = 0;
  specialScanID = 0;

  // relative to scheduler
  schedulerIndexTask = 0;
  schedulerIndexMotor = 0;
  for(uint8_t i=0; i<NETWORK_SCHEDULER_SIZE; i++)
  {
    if(i == 0) taskSchedule[i] = GetTemperature;
    else taskSchedule[i] = GetTorque;
  }
}

void Network::setup()
{
  //  Fault latch (used to store actual torque value (enable or disable)
  pinMode(FAULT_LATCH_OUT, INPUT);
  pinMode(FAULT_LATCH_SET,OUTPUT);    digitalWrite(FAULT_LATCH_SET, HIGH);
  pinMode(FAULT_LATCH_RST,OUTPUT);    digitalWrite(FAULT_LATCH_RST, HIGH);
  digitalWrite(FAULT_LATCH_RST, LOW); digitalWrite(FAULT_LATCH_RST, HIGH);

  setDefaultParameters(BROADCAST_ID);
  enableTorque(false);
  
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++)
  {
    int pos = SCS15.getPosition(i);
    goalPosMot[i] = 512;
    
    if(pos >= 0)
    {
      posMot[i] = pos;
      failMot[i] = 0;
    }
    else
    {
      failMot[i] = 255;
    }
    asm("wdr");
  }
}

void Network::loadFromEeprom()
{
  int address = 1;
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++,address++)
  {
    failMot[i] = EEPROM.read(address);
  }
  
  setDefaultParameters(BROADCAST_ID);
  pinMode(FAULT_LATCH_OUT, INPUT);
  enableTorque(!digitalRead(FAULT_LATCH_OUT));

  pinMode(FAULT_LATCH_SET,OUTPUT);    digitalWrite(FAULT_LATCH_SET, HIGH);
  pinMode(FAULT_LATCH_RST,OUTPUT);    digitalWrite(FAULT_LATCH_RST, HIGH);
  digitalWrite(FAULT_LATCH_RST, LOW); digitalWrite(FAULT_LATCH_RST, HIGH);
}

bool Network::enable()
{
  pinMode(NET_LATCH_OUT, INPUT);
  bool watchdogReset = !digitalRead(NET_LATCH_OUT);
  pinMode(EN_Network,OUTPUT);
  digitalWrite(EN_Network, LOW);
  digitalWrite(EN_Network, HIGH);

  if(digitalRead(NET_LATCH_OUT))
  {
    pinMode(NET_LATCH_OUT,OUTPUT);
    digitalWrite(NET_LATCH_OUT, LOW);    
  }
  
  return watchdogReset;
}


void Network::setLoopTime(unsigned long time)
{
  loopTime = time;
}

void Network::setDefaultParameters(uint8_t id)
{
  SCS15.setLimitTroque(id, 1023);
  SCS15.setSpeed(id, loopTime);
}

int Network::fixedFunction(int retry)
{
  // hack
  setDefaultParameters(BROADCAST_ID);
  
  // slave handling
  int dummy =  SLAVE.getRegister(0x01, S_CONFIG, 1);
  if(dummy >= 0) funnyControl = dummy;
  else funnyControl = 0x00;

  distance1 = SLAVE.getRegister(0x01, S_DISTANCE_1, 2);
  distance2 = SLAVE.getRegister(0x01, S_DISTANCE_2, 2);
  sorient_x = SLAVE.getFloatRegister(0x02, S_SPEED_ORIENTATION);
  orient_x =  SLAVE.getFloatRegister(0x02, S_ORIENTATION);
  

  // motor handling
  respondingIDN = 0;
  uint8_t maxNetworkFault = 4;
  for(int i=0; i<MAX_MOTOR_ON_CHANNEL && maxNetworkFault; i++)
  {
    if(failMot[i] == 0 || (failMot[i] < STOP_TRY_CONTACT_MOTOR && retry))
    {
      int pos = SCS15.getPosition(i);
      if(pos >= 1024)
      {
        // network problem !!!
      }
      else if(pos >= 0)
      {
        posMot[i] = pos;
        failMot[i] = 0;
        
        idsSend[respondingIDN] = i;
        posSend[respondingIDN] = goalPosMot[i];
        respondingIDN++;
      }
      else
      {
        failMot[i]++;
        retry--;
        maxNetworkFault--;
        if(failMot[i] >= STOP_TRY_CONTACT_MOTOR)
        {
          //motor disconnected
        }
      }
    }

    //  Send array of goal position
    //  motor buffer <= 64 bytes so we stop incrementing sync write index up to 20 and send a full packet at maximum size (64 byte message)
    if(respondingIDN == 20)
    {
      SCS15.syncSetPosition(idsSend, respondingIDN, posSend);
      respondingIDN = 0;
    } 
  }

  //  Send array of goal position remaning
  if(respondingIDN) SCS15.syncSetPosition(idsSend, respondingIDN, posSend); 
  
  return retry;
}

void Network::nodeScan()
{
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++,discoveryIndex++)
  {
    discoveryIndex %= MAX_MOTOR_ON_CHANNEL;
    
    if(failMot[discoveryIndex])
    {
      if(discoveryIndex < NUMBER_OF_SPECIAL_IDS)
      {
        if(specialScan)
        {
          specialScan--;
          discoveryIndex = NUMBER_OF_SPECIAL_IDS;
          continue;
        }
        else
        {
          specialScan = SPECIAL_SCAN_FREQUENCY;
          specialScanID++;
          specialScanID %= NUMBER_OF_SPECIAL_IDS;
          discoveryIndex = NUMBER_OF_SPECIAL_IDS;
          
          if(SCS15.ping(specialScanID))
          {
            setDefaultParameters(specialScanID);
            SCS15.enableTorque(specialScanID, false);
            if(specialScanID == 1 && failMot[0])
            {
              SCS15.setTemporaryID(specialScanID, 0);
              failMot[0] = 0;
            }
            else failMot[specialScanID] = 0;
          }
        }
      }
      else
      {
        if(SCS15.ping(discoveryIndex))
        {
          setDefaultParameters(discoveryIndex);
          SCS15.enableTorque(discoveryIndex, torque);
          failMot[discoveryIndex] = 0;
        }
        discoveryIndex++;
      }
      break;
    }
  }
}

void Network::taskScheduled()
{
  for(; schedulerIndexMotor<MAX_MOTOR_ON_CHANNEL; schedulerIndexMotor++)
  {
    if(!failMot[schedulerIndexMotor])
    {
      int data = 0;
      switch(taskSchedule[schedulerIndexTask])
      {
        case GetTorque:       data = SCS15.getTorque(schedulerIndexMotor);      break;
        case GetTemperature:  data = SCS15.getTemperature(schedulerIndexMotor); break;
        default: break;
      }

      if(data < 0) failMot[schedulerIndexMotor]++;
      else
      {
        switch(taskSchedule[schedulerIndexTask])
        {
          case GetTorque:       torqueMot[schedulerIndexMotor] = data;  break;
          case GetTemperature:  tempMot[schedulerIndexMotor] = data;    break;
          default: break;
        }
      }

      schedulerIndexMotor++;
      break;
    }
  }
  
  if(schedulerIndexMotor >= MAX_MOTOR_ON_CHANNEL)
  {
    schedulerIndexMotor = 0;
    schedulerIndexTask++;
    schedulerIndexTask %= NETWORK_SCHEDULER_SIZE;
  }
}





void Network::enableTorque(bool enable)
{
  torque = enable;
  SCS15.enableTorque(BROADCAST_ID, torque);
  if(!failMot[0]) SCS15.enableTorque(0x00, false);
  if(!failMot[1]) SCS15.enableTorque(0x01, false);
}

void Network::action(uint8_t id)
{
  SLAVE.action(id);
}

void Network::reset(uint8_t id)
{
  SLAVE.reset(id);
}

