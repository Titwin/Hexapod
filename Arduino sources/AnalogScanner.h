#include "Arduino.h"
#include "Configuration.h"

class AnalogScanner
{
  public:
    //  Default
    AnalogScanner();
    
    void initialize();
    //

    //  Public functions
    void start();
    void stop();
    void update(unsigned long stoptime);
    //

  protected:
    //  Attributes
    uint8_t scanIndex;
    //

    #define LOWPASS_FILTER 1
};


