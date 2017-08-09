#include "Watchdog.h"
#include "Network.h"

// extern variable for the save state sub-routine (search in code for more explanation of this variable usage and role)
extern uint8_t failMot[];
extern uint8_t boardControl;
extern Network NET;

Watchdog::Watchdog(uint8_t resetTime)
{
  prescaler = resetTime;  // set the prescaler value
  off();                  // turn of the watchdog to avoid unexpected and weird problem due to multiple reset at initialization
}

void Watchdog::off()
{
  wdt_disable();          // disable watchdog calling an Arduino function (for the hardware abstraction)
}

void Watchdog::on()
{
  wdt_enable(prescaler);        // set the new prescaler in the watchdog register
  WDTCSR  |= (1<<WDIE);         // start the watchdog clock
}

void Watchdog::setPrescaler(uint8_t resetTime)
{
  if(resetTime != prescaler)    // prevent to useless function call by checking the new prescaler value first
  {
    prescaler = resetTime;      // change prescaler value
    on();                       // turn on the watchdog
  }
}

ISR(WDT_vect)
{
  if(NET.getTorque())
  {
    digitalWrite(FAULT_LATCH_SET, LOW);
    digitalWrite(FAULT_LATCH_SET, HIGH);
  }
  
  EEPROM.update(0, boardControl);                             // write the board control byte at address 0 if needed (if not needed prevent EEPROM to useless write)
  int address = 1;                                            // initialize the adress counter
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++,address++)     
    EEPROM.update(address, failMot[i]);                       // write the failMot array into EEPROM and increment adress index
}

