#include "Configuration.h"

#include "Arduino.h"


class AnalogScanner
{
  public:
    //  Default
    AnalogScanner();
    
    void initialize();
    //

    //  Public functions
    void update(unsigned long stoptime);
    //

  protected:
    //  Attributes
    uint8_t scanIndex;
    //
};


