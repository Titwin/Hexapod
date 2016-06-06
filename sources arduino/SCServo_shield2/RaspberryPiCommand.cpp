#include "RaspberryPiCommand.h"

uint8_t state = 0;
uint8_t code;
int msgLength;
uint8_t crc;

uint8_t buffIndex;
uint8_t buff[60];

void parseRaspberryPiCommand(uint8_t c)
{
  if(state == 0 && c == 0xFF)
  {
    state = 1; buffIndex = 0; crc = 0;
    code = 0;  msgLength = 0;
  }
  else if(state == 1)
  {
    if(c == 0xFF)state = 2;
    else state = 0;
  }
  else if(state == 2)
  {
    code = c;
    crc = code;
    buffIndex = 0;
    
    if(code == Ping) {msgLength = 0; state = 4;}
    else if(code == GetMotorPosition) {msgLength = 0; state = 4;}
    else if(code == SetMotorPosition) {msgLength = 48; state = 3;}
    else if(code == EnableDisableTorque){msgLength = 1; state = 3;}
    else if(code == OpenUmbrella){msgLength = 0; state = 4;}
    else {state = 0; Serial.print((int)code);Serial.println(" ukn code");}
  }
  else if(state == 3)
  {
    buff[buffIndex] = c;
    buffIndex++;
    crc += c;
    if(buffIndex >= msgLength) state = 4;
  }
  else if(state ==4)
  {
    if(c == crc)
    {
      if(code == Ping) PingCb();
      else if(code == GetMotorPosition) GetMotorPositionCb();
      else if(code == SetMotorPosition) SetMotorPositionCb();
      else if(code == EnableDisableTorque) EnableDisableTorqueCb();
      else if(code == OpenUmbrella) OpenUmbrellaCb();
      else Serial.print("bsod");
    }
    else
    {
      Serial.print("crc:");
      Serial.print((int)crc);
      Serial.write('/');
      Serial.println((int)c);
    }
    
    state = 0;
    buffIndex = 0;
    msgLength = 0;
    code = 0;
  }
}

void PingCb()
{
  //Serial.println("ping");

  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(Ping); crc = Ping;
  Serial1.write(crc);
}
void GetMotorPositionCb()
{
  //Serial.println("get");
          
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(GetMotorPosition);
  crc = GetMotorPosition;
  
  Serial1.write((uint8_t*)MotorPosition,48);
  for(int i=0;i<24;i++)
    crc += (uint8_t)(MotorPosition[i]>>8) + (uint8_t)(MotorPosition[i]&0xFF);
    
  Serial1.write((uint8_t*)&distance,6);
  for(int i=0;i<3;i++)
    crc += (uint8_t)(distance[i]>>8) + (uint8_t)(distance[i]&0xFF);
    
  Serial1.write(crc);
}
void SetMotorPositionCb()
{
  debugLong++;
  //Serial.println("set");
  memcpy((uint8_t*)TargetPosition,buff,48);
}
void EnableDisableTorqueCb()
{
  //Serial.println("toogle torque");
  if(buff[0] == 0x01) SERVO.EnableTorque(0xfe,1);
  else SERVO.EnableTorque(0xfe,0);

  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(EnableDisableTorque); crc = EnableDisableTorque;
  Serial1.write(crc);
}
void OpenUmbrellaCb()
{
  openUmbrela = true;

  //Serial.println("moving optionnal servo 1");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(OpenUmbrella); crc = OpenUmbrella;
  Serial1.write(crc);
}


