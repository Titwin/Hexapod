#include "Arduino.h"

#include <avr/wdt.h>

/*
 * Please read :
 * http://forum.arduino.cc/index.php?topic=401628.msg2782592#msg2782592
 * in order to use the watchdog on amega32u4
 */

class Watchdog
{
  public:
    //  Default
    Watchdog(uint8_t resetTime = WDTO_120MS);

    //  Public functions
    void on();
    void off();
    void setPrescaler(uint8_t resetTime);
    inline void reset() { asm("wdr"); };
    //
    
  protected:
    //  Attributes
    uint8_t prescaler;  //!< The watchdog prescaler (the starting value of the counter)
    //
};

//  Interrupt routine prototype
ISR(WDT_vect);
//
