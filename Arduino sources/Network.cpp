#include "Network.h"


extern SCS15Controller SCS15;

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
  setDefaultParameters(0xFE);
  respondingIDN = 0;
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++)
  {
    int pos = SCS15.getPosition(i);
    if(pos >= 0)
    {
      posMot[i] = pos;
      goalPosMot[i] = pos;
      failMot[i] = 0;

      idsSend[respondingIDN] = i;
      posSend[respondingIDN] = goalPosMot[i];
      respondingIDN++;
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
    failMot[i] = EEPROM.read(address);
}

void Network::setLoopTime(unsigned long time)
{
  loopTime = time;
}

void Network::setDefaultParameters(uint8_t id, bool enableTorque)
{
  SCS15.setLimitTroque(id, 128);
  SCS15.setSpeed(id, loopTime);
  SCS15.enableTorque(id, enableTorque);
}

int Network::fixedFunction(int retry)
{
  respondingIDN = 0;
  uint8_t maxNetworkFault = 4;
  for(int i=0; i<MAX_MOTOR_ON_CHANNEL && maxNetworkFault; i++)
  {
    if(failMot[i] == 0 || (failMot[i] < STOP_TRY_CONTACT_MOTOR && retry))
    {
      int pos = SCS15.getPosition(i);
      if(pos >= 0)
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
  }

  //  Send array of goal position
  SCS15.syncSetPosition(idsSend,respondingIDN,posSend);
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
            setDefaultParameters(specialScanID, false);
            failMot[specialScanID] = 0;
          }
        }
      }
      else
      {
        if(SCS15.ping(discoveryIndex))
        {
          setDefaultParameters(discoveryIndex);
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


