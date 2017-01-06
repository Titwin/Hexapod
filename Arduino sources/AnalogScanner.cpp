#include "AnalogScanner.h"


const int analogInPin[MAX_ANALOG_INPUT] = {A0,A1,A2,A3, A4,A5,A6,A7, A8,A9,A10,A11, A12,A13,A14,A15};
extern uint16_t analogIn[];


AnalogScanner::AnalogScanner(){}

void AnalogScanner::initialize()
{
  //  initialize Analog input and buffer
  for(uint8_t i=0; i<MAX_ANALOG_INPUT; i++)
  {
    pinMode(analogInPin[i], INPUT_PULLUP);
    analogIn[i] = 1024;
  }
  scanIndex = 0;
}

void AnalogScanner::update(unsigned long stoptime)
{
  for(uint8_t i=0; i<MAX_ANALOG_INPUT; i++)
  {
    if(millis() > stoptime) return;
    analogIn[scanIndex]  = analogRead(scanIndex);
    scanIndex++;
    scanIndex %= MAX_ANALOG_INPUT;
  }
}

