#include <Servo.h>

#include "SCS15Controller.h"
#include "SlaveController.h"

SCS15Controller SCS15;
SlaveController Slaves;
Servo chineseServo;

void setup()
{
  Network.begin(1000000);
  Network.setTimeout(10);
  enableRX();

  Serial.begin(9600);
  while(!Serial);  
  Serial.setTimeout(6000);
  Serial.println(F("Hello friend !"));

  chineseServo.attach(9);
}


int askUserFor(String s)
{
  Serial.print(F(">> enter a "));
  Serial.print(s);
  Serial.println(F(" please .."));
    while(Serial.available())
      Serial.read();
  
  while(!Serial.available()) asm("wdr");
  String message = Serial.readStringUntil('\n');
  int result = message.toInt();
  Serial.print("<< '");
  Serial.print(result);
  Serial.println("'");
  return result;
}


void loop()
{
  chineseServo.write(90);
  delay(100);
  
  if(Serial.available())
  {
    String message = Serial.readStringUntil('\n');
    if(message.equalsIgnoreCase("help"))
    {
      Serial.println(F(">> list of available commands :"));
      Serial.println(F("  help"));
      Serial.println(F("  start"));
      Serial.println(F("  stop"));
      Serial.println(F("  action"));
      Serial.println(F("  scan"));
      Serial.println(F("  band pass test"));
      Serial.println(F("  reset motors"));
      Serial.println(F("  reset slaves"));
      Serial.println();
      Serial.println(F("  set motor id"));
      Serial.println(F("  ping motor"));
      Serial.println(F("  print motor"));
      Serial.println(F("  set motor pos"));
      Serial.println();
      Serial.println(F("  set slave id"));
      Serial.println(F("  ping slave"));
      Serial.println(F("  print slave"));
      Serial.println(F("  set slave color"));
      Serial.println(F("  set slave blink mode"));
      Serial.println(F("  set slave thresholds"));
    }
    else if(message.equalsIgnoreCase("stop"))
    {
      SCS15.enableTorque(BROADCAST_ID, 0);
      Serial.println(F(">> disable torque .."));
      return;
    }
    else if(message.equalsIgnoreCase("start"))
    {
      SCS15.enableTorque(BROADCAST_ID, 1);
      Serial.println(F(">> enable torque .."));
      return;
    }
    else if(message.equalsIgnoreCase("action"))
    {
      Slaves.action(BROADCAST_ID);
      Serial.println(F(">> action ! .."));
      return;
    }
    else if(message.equalsIgnoreCase("reset motors"))
    {
      Serial.println(F(">> reset position .."));
      SCS15.enableTorque(BROADCAST_ID, 1);
      SCS15.setPosition(BROADCAST_ID, 512);
    }
    else if(message.equalsIgnoreCase("reset slaves"))
    {
      Serial.println(F(">> reset slaves .."));
      Slaves.reset(BROADCAST_ID);
      delay(1000);
    }
    else if(message.equalsIgnoreCase("set motor id"))
    {
      int oldid = askUserFor("current id");
      int newid = askUserFor("new id");
      if(oldid > 0 && oldid <= BROADCAST_ID && newid > 0 && newid < BROADCAST_ID)
      {
        int error = SCS15.setPermanentID(oldid, newid);
        if(error)
        {
          Serial.print(F("error : "));
          Serial.println(error);
        }
        else Serial.println(F(">> success"));
      }
      else Serial.println(F("error : wrong id entered."));
    }
    else if(message.equalsIgnoreCase("set slave id"))
    {
      int oldid = askUserFor("current id");
      int newid = askUserFor("new id");
      if(oldid > 0 && oldid <= BROADCAST_ID && newid > 0 && newid < BROADCAST_ID)
      {
        int error = Slaves.setPermanentID(oldid, newid);
        if(error)
        {
          Serial.print(">> error : ");
          Serial.println(error);
        }
        else Serial.println(F(">> success"));
      }
      else Serial.println(F(">> wrong id entered."));
    }
    else if(message.equalsIgnoreCase("ping motor"))
    {
      int pingId = askUserFor("id");
      if(SCS15.ping(pingId)) Serial.println(F(">> ping true"));
      else Serial.println(F("motor not responding"));
    }
    else if(message.equalsIgnoreCase("print motor"))
    {
      int pingId = askUserFor("id");
      if(!SCS15.debug(pingId))
        Serial.println(F("motor not resonding"));
    }
    else if(message.equalsIgnoreCase("ping slave"))
    {
      int pingId = askUserFor("id");
      if(Slaves.ping(pingId)) Serial.println("true");
      else Serial.println(F("slave not responding"));
    }
    else if(message.equalsIgnoreCase("print slave"))
    {
      int pingId = askUserFor("id");
      if(!Slaves.debug(pingId))
        Serial.println(F("slave not resonding"));
    }
    else if(message.equalsIgnoreCase("scan"))
    {
      Serial.println(F(">> motor id list : "));
      for(int i=0; i<BROADCAST_ID; i++)
      {
        if(SCS15.ping(i))
        {
          Serial.print(i);
          Serial.write(' ');
        }
      }
      Serial.println();
      Serial.println(F("   slave id list : "));
      for(int i=0; i<BROADCAST_ID; i++)
      {
        if(Slaves.ping(i))
        {
          Serial.print(i);
          Serial.write(' ');
        }
      }
      Serial.println();
      return;
    }
    else if(message.equalsIgnoreCase("band pass test"))
    {
      uint8_t motorList[254]; uint8_t motorCounter = 0;
      uint8_t slaveList[254]; uint8_t slaveCounter = 0;
      Serial.println(F(">> motor id list : "));
      for(int i=0; i<BROADCAST_ID; i++)
      {
        if(SCS15.ping(i))
        {
          Serial.print(i);
          Serial.write(' ');
          motorList[motorCounter] = i;
          motorCounter++;
        }
      }
      Serial.println();
      Serial.println(F("   slave id list : "));
      for(int i=0; i<BROADCAST_ID; i++)
      {
        if(Slaves.ping(i))
        {
          Serial.print(i);
          Serial.write(' ');
          slaveList[slaveCounter] = i;
          slaveCounter++;
        }
      }
      Serial.println();
      Serial.print(F("motors : "));    Serial.print((int)motorCounter);
      Serial.print(F(", slaves : ")); Serial.println((int)slaveCounter);

      unsigned long start = millis();
      while(millis()-start < 5012 && motorCounter + slaveCounter > 0)
      {
        for(uint8_t i=0; i<motorCounter; i++)
          SCS15.ping(motorList[i]);
        for(uint8_t i=0; i<slaveCounter; i++)
          Slaves.ping(slaveList[i]);
        delay(1);
      }
      Serial.println(F(">>end"));
      return;
    }
    else if(message.equalsIgnoreCase("set motor pos"))
    {
      int motid = askUserFor("id");
      if(motid && motid <= BROADCAST_ID)
      {
        int pos = askUserFor("angle or position");
        if(pos > 1024) pos = 1024;
        else if(pos<0) pos = 0;
        
        SCS15.enableTorque(motid, 1);
        SCS15.setPosition(motid, pos);
      }
      else Serial.println(F(">> invalid motor id"));
    }
    else if(message.equalsIgnoreCase("set slave color"))
    {
      int id = askUserFor("slave id");
      if(id > 0 && id <= BROADCAST_ID)
      {
        uint8_t color[3];
        color[0] = askUserFor("red brightness");
        color[1] = askUserFor("green brightness");
        color[2] = askUserFor("blue brightness");
        
        if(Slaves.setRegister(id, REG_LED_RED, 1, &color[0])!= 6) Serial.println("error red");
        else if(Slaves.setRegister(id, REG_LED_GREEN, 1, &color[1])!= 6) Serial.println("error green");
        else if(Slaves.setRegister(id, REG_LED_BLUE, 1, &color[2])!= 6) Serial.println("error blue");
        else Serial.println(F(">> success"));
      }
    }
    else if(message.equalsIgnoreCase("set slave blink mode"))
    {
      int id = askUserFor("slave id");
      if(id > 0 && id <= BROADCAST_ID)
      {
        uint8_t mode = (askUserFor("blink mode")&0x03)|(1<<EEPROM_LOCK);
        if(Slaves.setRegister(id, REG_STATE, 1, &mode)!= 6) Serial.println("error changing mode");
        else Serial.println(F(">> success"));
      }
    }
    else if(message.equalsIgnoreCase("set slave thresholds"))
    {
      int id = askUserFor("slave id");
      if(id > 0 && id <= BROADCAST_ID)
      {
        int distance = askUserFor("distance threshold");
        int force = askUserFor("force threshold");
      
        Network.setTimeout(20);
        uint8_t error = 0;
        for(;;)
        {
          uint8_t locker = 0<<EEPROM_LOCK;
          if(Slaves.setRegister(id, REG_STATE, 1, &locker)!= 6)
          {
            Serial.println(F("error while unlocking EEPROM"));
            break;
          }
          if(Slaves.setRegister(id, REG_SIZE + EEPROM_DISTANCE_THSD_H, 2, (uint8_t*)&distance) != 6)
          {
            Serial.println(F("error while changing distance threshold"));
            break;
          }
          if(Slaves.setRegister(id, REG_SIZE + EEPROM_FORCE_THSD_H, 2, (uint8_t*)&force) != 6)
          {
            Serial.println(F("error while changing force threshold"));
            break;
          }
          locker = 1<<EEPROM_LOCK;
          if(Slaves.setRegister(id, REG_STATE, 1, &locker)!= 6)
          {
            Serial.println(F("error while re-locking EEPROM"));
            break;
          }
          Serial.println(F(">> success"));
          break;
        }
        Network.setTimeout(3);
      }
      else Serial.println(">> wrong id entered.");
    }
    else
    {
      Serial.print(F(">> unknown command:\n\t'"));
      Serial.print(message);
      Serial.println("'");
      Serial.println(F(">> enter 'help' for a list of available command"));
    }
  }
}
