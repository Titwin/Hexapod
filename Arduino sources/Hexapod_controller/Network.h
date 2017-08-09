#include "Configuration.h"

#include "Arduino.h"
#include "SCS15Controller.h"
#include "SlaveController.h"

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
    bool enable();
    //

    //  Public functions
    void setLoopTime(unsigned long time);
    int fixedFunction(int retry = 1);
    void nodeScan();
    void taskScheduled();
    void enableTorque(bool enable);
    bool getTorque(){return torque;};
    //

    //
    void action(uint8_t id);
    void reset(uint8_t id);
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
    void setDefaultParameters(uint8_t id);
    //
    
    //  Attributes
    unsigned long loopTime;

    // relative to fail
    unsigned int respondingIDN;

    // relative to torque
    bool torque;

    // relative to discovery scan
    uint8_t discoveryIndex;
    int specialScan;
    uint8_t specialScanID;

    // relative to sceduller
    Task taskSchedule[NETWORK_SCHEDULER_SIZE];
    uint8_t schedulerIndexTask;
    uint8_t schedulerIndexMotor;

    #define NUMBER_OF_SPECIAL_IDS 2
    #define SPECIAL_SCAN_FREQUENCY 50
    #define STOP_TRY_CONTACT_MOTOR 5
};

