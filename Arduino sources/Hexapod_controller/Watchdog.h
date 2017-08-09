#include "Configuration.h"

#include "Arduino.h"


#include <avr/wdt.h>
#include <EEPROM.h>

/*! \class Watchdog
 *  \brief A wrapper class to the arduino watchdog
 *
 *  Since the robot can enter in some deadlock it's important to enable the watchdog 
 *  to be sure that the main function of the robot are still running in time.
 *  
 *  This wrapper function define some public function to call if you want to enable, disable, 
 *  or reset the watchdog, but it also define the interruption routine to call if a reset occur
 *  due to a watchdog counter overflow (you have to change code in the Watchdog.cpp file to change
 *  routine).
 *
 */
class Watchdog
{
  public:
    //  Functions
    /*!
     *  \brief Constructor
     *  
     *  The default value of the resetTime parameter correspond to a 120ms time before reset.
     *  At the initialization the watchdog is turn off to prevent some problem at startup.
     *  You have to turn it on at the end of the Setup function from Arduino API (or not).
     *  
     *  \param resetTime : the starting counter value. Has to be a value define by Arduino as a macro to be valid
     */
    Watchdog(uint8_t resetTime = WDTO_120MS);

    /*!
     *  \brief Turn on the watchdog
     */
    void on();

    /*!
     *  \brief Turn off the watchdog
     */
    void off();

    /*!
     *  \brief Change the watchdog prescaler
     *  
     *  If the new value is different to the old one the watchdog is turned on after this function     
     *    
     *  \param resetTime : the starting counter value. Has to be a value define by Arduino as a macro to be valid
     */
    void setPrescaler(uint8_t resetTime);

    /*!
     *  \brief Reset the watchdog counter (to prevent board reset)
     *  
     *  Call this function once per loop, and before an intensive computing.
     */
    inline void reset() { asm("wdr"); };
    //
    
  protected:
    //  Attributes
    uint8_t prescaler;  //!< The watchdog prescaler (the starting value of the counter)
    //
};

//  Interrupt routine prototype
/*!
 *  \brief The interuption sub-routine
 *  
 *  This function is automaticaly called when a counter overflow occur, and when it finished a reset occur. 
 *  Now the function save in EEPROM the current FailMot array to keep track of the net on the slave network,
 *  and set the flag CTRL_WDT_SHUTDOWN in boardControl byte before to save it in EEPROM.
 *  
 *  After a reset the Setup function read the boardControl byte save in EEPROM to see if the reset is due to a watchdog reset.
 *  If it's the case the programme reload the FailMot array to save time scanning the network
 *  
 */
ISR(WDT_vect);
//
