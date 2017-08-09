#include "AnalogScanner.h"


const int analogInPin[ANALOG_COUNT] = {Sens_Bat, Sens_Front, Sens_Back, Sens_Leg0, Sens_Leg1, Sens_Leg2, Sens_Leg3, Sens_Leg4, Sens_Leg5};
extern int analogIn[];


AnalogScanner::AnalogScanner(){}

void AnalogScanner::initialize()
{
  //  initialize Analog input and buffer
  for(uint8_t i=0; i<9; i++)
  {
    pinMode(analogInPin[i], INPUT);
    analogIn[i] = 0;
  }
  scanIndex = 0;
}

void AnalogScanner::update(unsigned long stoptime)
{
  for(uint8_t i=0; i<ANALOG_COUNT; i++)
  {
    if(millis() > stoptime) return;
    if(scanIndex == 0)
    {
      analogReference(DEFAULT);
      analogRead(analogInPin[scanIndex]);
      analogIn[scanIndex]  = analogRead(analogInPin[scanIndex]);
      analogReference(INTERNAL1V1);
      analogRead(analogInPin[scanIndex]);
    }
    else analogIn[scanIndex]  = analogRead(analogInPin[scanIndex]);
    scanIndex++;
    scanIndex %= ANALOG_COUNT;
  }
}

