#include "Arduino.h"
#include "Configuration.h"

#include <avr/wdt.h>
#include <EEPROM.h>

class Watchdog
{
  public:
    //  Functions
    Watchdog(uint8_t resetTime = WDTO_120MS);

    void on();
    void off();
    inline void reset() { asm("wdr"); };
    //
    
  protected:
    //  Attributes
    uint8_t prescaler;
    //
};

//  Interrupt routine prototype
ISR(WDT_vect);
//
