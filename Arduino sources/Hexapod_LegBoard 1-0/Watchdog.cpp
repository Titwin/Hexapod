#include "Watchdog.h"

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

}

