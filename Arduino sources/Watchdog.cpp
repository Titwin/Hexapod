#include "Watchdog.h"

extern uint8_t failMot[];
extern uint8_t boardControl;

Watchdog::Watchdog(uint8_t resetTime)
{
  prescaler = resetTime;
  off();
}

void Watchdog::off()
{
  wdt_disable();
}

void Watchdog::on()
{
  wdt_enable(prescaler);
  WDTCSR  |= (1<<WDIE);
}

ISR(WDT_vect)
{
  boardControl |= CTRL_WDT_SHUTDOWN;
  int address = 0;
  EEPROM.update(address, boardControl);
  address++;
  
  for(uint8_t i=0; i<MAX_MOTOR_ON_CHANNEL; i++,address++)
    EEPROM.update(address, failMot[i]);
}

