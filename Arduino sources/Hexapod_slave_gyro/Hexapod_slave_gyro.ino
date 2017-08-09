#include "Arduino.h"
#include "Configuration.h"
#include "NetworkController.h"

#include <Wire.h>
#include <L3G.h>


//  Global variable initialization
NetworkController NETWORK;
L3G gyro;

unsigned long startTime;

float speed_x;
float orient_x;
uint16_t samples;
//


//  Initialization
void setup()
{
  //Serial.begin(250000);
  NETWORK.initialize();
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  if (!gyro.init())
  {
    while(1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  else digitalWrite(LED_BUILTIN, LOW);
  //
 
  // 0x00 = 0b00000000
  // FS = 00 (+/- 250 dps full scale)
  gyro.writeReg(L3G::CTRL_REG4, 0x00);
  
  // 0x6F = 0b01101111
  //   DR = 11 (800 Hz ODR);
  // ? BW = 10 (50 Hz bandwidth);
  //   PD = 1 (normal mode);
  //   Xen = 1 (x axis enabled)
  gyro.writeReg(L3G::CTRL_REG1, 0b11101010);

  samples = 0;
}
//



int16_t dummy;
int16_t offset_x;
int32_t sumx;

//  Program
void loop()
{
  //  Get new values
  startTime = millis();
  gyro.read();
  
  if(samples == 1000)
  {
    digitalWrite(LED_BUILTIN, LOW);
    //  Integration
    dummy = gyro.g.x - offset_x;
    if(dummy < -70) speed_x = dummy*0.0076;
    else if(dummy > 70) speed_x = dummy*0.0076;
    else speed_x = 0.f;
    orient_x += speed_x;    
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if(samples == 0)
    {
      sumx = 0;
      orient_x = 0;
    }
    samples++;
    sumx += gyro.g.x;
    if(samples == 1000) offset_x = sumx/1000;
  }
  //Serial.println(speed_x);
  
  //  Network managment
  while(millis() - startTime < 2)
  {
    if(Serial.available())
    {
      int c = Serial.read();
      if(c >= 0) NETWORK.parseChar(c);     
    }
  }
}
//



