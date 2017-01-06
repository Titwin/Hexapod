#include "Arduino.h"
#include "SCS15Controller.h"
#include "Configuration.h"

#include <EEPROM.h>

class Network
{
  public:
    //  Default
    Network();
    //
    
    //  Different initialization functions
    void setup();
    void loadFromEeprom();
    //

    //  Public functions
    void setLoopTime(unsigned long time);
    int fixedFunction(int retry = 1);
    void nodeScan();
    void taskScheduled();
    //

  protected:
    //  Micelenious
    enum Task
    {
      None,
      //GetSpeed,
      GetTorque,
      GetTemperature
    };
    //

    //  Functions
    void setDefaultParameters(uint8_t id, bool enableTorque = false);
    //
    
    //  Attributes
    unsigned long loopTime;

    // relative to fail
    unsigned int respondingIDN;

    // relative to discovery scan
    uint8_t discoveryIndex;
    int specialScan;
    uint8_t specialScanID;

    // relative to sceduller
    Task taskSchedule[NETWORK_SCHEDULER_SIZE];
    uint8_t schedulerIndexTask;
    uint8_t schedulerIndexMotor;

    #define NUMBER_OF_SPECIAL_IDS 2
    #define SPECIAL_SCAN_FREQUENCY 60
    #define STOP_TRY_CONTACT_MOTOR 5
};

