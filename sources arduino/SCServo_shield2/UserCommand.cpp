#include "UserCommand.h"

int askUserFor(String s)
{
  Serial.print(">> enter a ");
  Serial.print(s);
  Serial.println(" please ..");
    while(Serial.available())
      Serial.read();
  
  while(!Serial.available()) asm("wdr");
  String message = Serial.readStringUntil('\n');
  int result = message.toInt();
  Serial.print(">> '");
  Serial.print(result);
  Serial.println("'");
  return result;
}

void parseUserCommand()
{
  if(!Serial.available()) return;
  
  String message = Serial.readStringUntil('\n');
  if(message.equalsIgnoreCase("stop"))
  {
    SERVO.EnableTorque(0xfe,0);
    Serial.println(">> disable torque ..");
    return;
  }
  else if(message.equalsIgnoreCase("start"))
  {
    SERVO.EnableTorque(0xfe,1);
    Serial.println(">> enable torque ..");
    return;
  }
  else if(message.equalsIgnoreCase("reset"))
  {
    Serial.println(">> reset position ..");
    SERVO.EnableTorque(0xfe,1);
    SERVO.WritePos(0xfe, 512, 2000);
  }
  else if(message.equalsIgnoreCase("set id"))
  {
    int newid = askUserFor("id");
    if(newid > 0 && newid < 0xfe)
    {
      SERVO.LockEprom(0xfe, 0);
      SERVO.WriteID(1,newid);
      SERVO.LockEprom(0xfe, 1);
      
      SERVO.EnableTorque(newid,1);
      SERVO.WritePos(newid, 512, 1000);
      delay(5000);
      
      SERVO.EnableTorque(newid,0);    
    }
    else
      Serial.println(">> wrong id entered.");
  }
  else if(message.equalsIgnoreCase("ping"))
  {
    int pingId = askUserFor("id");
    if(SERVO.ReadTemper(pingId) >= 0)
      Serial.println("true");
    else
      Serial.println("motor not responding");
  }
  else if(message.equalsIgnoreCase("mapping"))
  {
    Serial.println(">> mapping network ..");
    Serial.print("List of id used : { ");
    int nb_motor = 0;
    for(uint8_t i=0;i<254;i++)
    {
      if(SERVO.ReadTemper(i) >= 0)
      {
        Serial.print((int)i);
        Serial.print(", ");
        nb_motor++;
      }
      asm("wdr");
    }
    Serial.print("}\nNumber of motor on network : ");
    Serial.println(nb_motor);
    Serial.println(">> end");
  }
  else if(message.equalsIgnoreCase("read pos"))
  {
    Serial.println(">> reading position ..");
    int pos;
    for(uint8_t i=0;i<254;i++)
    {
      pos = SERVO.ReadPos(i);
      if(pos >= 0)
      {
        Serial.print(pos);
        Serial.write(',');
      }
      asm("wdr");
    }
    Serial.println(">> end");
  }
  else if(message.equalsIgnoreCase("set pos"))
  {
    int posid = askUserFor("id");
    if(posid >= 0 && posid < 5)
    {
      SERVO.EnableTorque(0xfe,1);
      for(uint8_t i=0; i<24; i++)
        SERVO.WritePos(i+2,(int) (pgm_read_word_near(RecordedPos +24*posid + i)), 2000);
    }
    else Serial.println(">> invalid position code");
  }
  else if(message.equalsIgnoreCase("set mot pos"))
  {
    int motid = askUserFor("motor id");
    if(motid < 254)
    {
      int pos = askUserFor("angle");
      if(pos>1024) pos = 1024;
      else if(pos<0) pos = 0;
      
      SERVO.EnableTorque(motid,1);
      SERVO.WritePos(motid,pos,2000);
    }
    else Serial.println(">> invalid motor id");
  }
  else if(message.equalsIgnoreCase("get mot pos"))
  {
    int motid = askUserFor("motor id");
    if(motid < 254)
    {
      int pos = SERVO.ReadPos(motid);
      Serial.print("angle : ");
      if(pos >= 0) Serial.println(pos);
      else Serial.println("read error");
    }
    else Serial.println(">> invalid motor id");
  }
  else if(message.equalsIgnoreCase("walk"))
  {
    int speed = askUserFor("speed");
    if(speed<200) { speed = 200; Serial.println(">> speed clamped to 200"); }
    int step = askUserFor("nuùmber of step");
    if(step>15) { step = 15; Serial.println(">> step clamped to 15"); }
    
    SERVO.EnableTorque(0xfe,1);
    for(uint8_t i=0; i<24; i++)
      SERVO.WritePos(i+2,(int) (pgm_read_word_near(RecordedPos + i)), 2000);
    delay(speed);
    
    for(uint8_t i=0;i<step;i++)
    {
      for(uint8_t j=1; j<5; j++)
      {
        for(uint8_t k=0; k<24; k++)
          SERVO.WritePos(k+2,(int) (pgm_read_word_near(RecordedPos +24*j + k)), speed);
        delay(speed);
      }
      Serial.println("one step futher");
    }
  }
  else
  {
    Serial.print(">> unknown command:\n\t'");
    Serial.print(message);
    Serial.println("'");
  }

  //delay(1000);
  message = "";
  while(Serial.available())
    Serial.read();
}







