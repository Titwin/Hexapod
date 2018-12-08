#define NO_SERIAL_1

#include "NetworkController.h"

#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

#include "Configuration.h"
#include "Watchdog.h"

#define LED_BLINK_TIME 500  // in ms
#define LED_PHASE_COUNT 4
#define USB_HOST_CONNECTED (USBSTA&(1<<VBUS) && UDADDR&(1<<ADDEN))


//  Global variable initialization
Watchdog WDOG(WDTO_60MS);
NetworkController NETWORK;
VL53L0X distanceSensor;
uint8_t registerMap[REG_SIZE + EEPROM_SIZE];
uint16_t distanceThreshold;
uint16_t forceThreshold;
//


//  Initialization
unsigned long timer = 0;
void setup()
{
  //  Type & version
  EEPROM.update(EEPROM_SLAVE_TYPE, LEGBOARD);
  EEPROM.update(EEPROM_VERSION, 0x03);

  { //  LED, status bytes & EEPROM load
    //  LED setup
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN,   LOW);
    pinMode(LED_Red,   OUTPUT);   analogWrite(LED_Red,   200);
    pinMode(LED_Green, OUTPUT);   analogWrite(LED_Green, 200);
    pinMode(LED_Blue,  OUTPUT);   analogWrite(LED_Blue,  200);
    registerMap[REG_LED_RED] = 50;
    registerMap[REG_LED_GREEN] = 0;
    registerMap[REG_LED_BLUE] = 0;
  
    
    //  status bytes & EEPROM load
    registerMap[REG_STATE] = (1<<EEPROM_LOCK);
    registerMap[REG_SIZE + EEPROM_ID] = EEPROM.read(EEPROM_ID);
    registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H] = EEPROM.read(EEPROM_DISTANCE_THSD_H);
    registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = EEPROM.read(EEPROM_DISTANCE_THSD_L);
    registerMap[REG_SIZE + EEPROM_FORCE_THSD_H] = EEPROM.read(EEPROM_FORCE_THSD_H);
    registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = EEPROM.read(EEPROM_FORCE_THSD_L);
    registerMap[REG_SIZE + EEPROM_SLAVE_TYPE] = EEPROM.read(EEPROM_SLAVE_TYPE);
    registerMap[REG_SIZE + EEPROM_VERSION] = EEPROM.read(EEPROM_VERSION);
    distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
    forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];  
  }

  { //  Nework, serials & Shield contacts
    //  Serials connections
    if(registerMap[REG_SIZE + EEPROM_ID] >= 0xFE || registerMap[REG_SIZE + EEPROM_ID] == 0x00)
    {
      registerMap[REG_SIZE + EEPROM_ID] = 0x01;
      EEPROM.update(EEPROM_ID, 0x01);
    }
    NETWORK.initialize(registerMap[REG_SIZE + EEPROM_ID]);
    NETWORK.assignCallback(NC_INST_READ, &callbackRead);
    NETWORK.assignCallback(NC_INST_WRITE, &callbackWrite);
    NETWORK.assignCallback(NC_INST_ACTION, &callbackAction);
    NETWORK.assignCallback(NC_INST_AUTOCALIBRATE, &callbackAutocalibrate);
    Serial.begin(250000);
  
  
    //  Shield contacts
    pinMode(SHIELD_A, INPUT_PULLUP);
    pinMode(SHIELD_B, INPUT_PULLUP);
    pinMode(SHIELD_C, INPUT_PULLUP);
    pinMode(SHIELD_D, INPUT_PULLUP);
    pinMode(SHIELD_E, INPUT_PULLUP);
    registerMap[REG_SHIELD] = 0x00;  
  }

  { //  Force & distance sensors
    //  Force sensor
    pinMode(FORCE, INPUT);
    registerMap[REG_FORCE_H] = 0xFF;
    registerMap[REG_FORCE_L] = 0xFF;
  
  
    //  distance sensor
    Wire.begin();
    delay(50);
    pinMode(VLX_RDY, INPUT_PULLUP);
    registerMap[REG_DISTANCE_H] = 0xFF;
    registerMap[REG_DISTANCE_L] = 0xFF;
    if(!distanceSensor.init())
      registerMap[REG_STATE] &= ~(1<<VLX_OK);
    else
    {
      registerMap[REG_STATE] |= (1<<VLX_OK);
      delay(100);
      distanceSensor.setTimeout(500);
      distanceSensor.setSignalRateLimit(0.1);
      distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    
      delay(100);
      distanceSensor.startContinuous();
    }  
  }

  //  start
  timer = millis();
  WDOG.on();
}
//


//  Program
int framecounter = 0;
uint8_t ledPhase = 0;
uint16_t distance = 0xFFFF;
uint16_t force;
uint16_t vbatt;

void loop()
{
  //  begin && USB connection detect
  WDOG.reset();
  if(USB_HOST_CONNECTED) registerMap[REG_STATE] |= (1<<SERIAL_USB);
  else registerMap[REG_STATE] &= ~(1<<SERIAL_USB);
  if(framecounter > LED_BLINK_TIME)
  {
    framecounter = 0;
    ledPhase = (ledPhase +1)%LED_PHASE_COUNT;
  }

  //  network
  NETWORK.update();

  { //  shield state changes & compute Thresholds
    //  shield state changes
    registerMap[REG_SHIELD] = 0x00;
    if(digitalRead(SHIELD_A) == LOW) registerMap[REG_SHIELD] |= 1<<0;
    if(digitalRead(SHIELD_C) == LOW) registerMap[REG_SHIELD] |= 1<<1;
    if(digitalRead(SHIELD_B) == LOW) registerMap[REG_SHIELD] |= 1<<2;
    if(digitalRead(SHIELD_D) == LOW) registerMap[REG_SHIELD] |= 1<<3;
    if(digitalRead(SHIELD_E) == LOW) registerMap[REG_SHIELD] |= 1<<4;
    if(registerMap[REG_SHIELD]) registerMap[REG_STATE] |= 1<<SHIELD_CONTACT;
    else registerMap[REG_STATE] &= ~(1<<SHIELD_CONTACT);
  
    
    //  Compute Thresholds
    distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
    forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];  
  }

  { //  Read distance, force & Vbatt
    //  read distance
    if((registerMap[REG_STATE]&(1<<VLX_OK)) && digitalRead(VLX_RDY) == LOW)
    {
      distance = distanceSensor.fastReadRange();
      registerMap[REG_DISTANCE_H] = distance>>8;
      registerMap[REG_DISTANCE_L] = distance&0xFF;
  
      if(distance < distanceThreshold)
        registerMap[REG_STATE] |= 1<<DISTANCE_THSD;
      else registerMap[REG_STATE] &= ~(1<<DISTANCE_THSD);
    }
  
    //  read force
    force = analogRead(FORCE);
    registerMap[REG_FORCE_H] = force>>8;
    registerMap[REG_FORCE_L] = force&0xFF;
    if(force > forceThreshold)
      registerMap[REG_STATE] |= 1<<FORCE_THSD;
    else registerMap[REG_STATE] &= ~(1<<FORCE_THSD);
  
  
    //  read battery tension
    vbatt = analogRead(VBATT);  
  }

  { //  RGB LED color change
    //  LED
    if(framecounter <= (LED_BLINK_TIME/4)*((registerMap[REG_STATE]&0x03) + 1)) // turn light on
    {
      analogWrite(LED_Red,   255-registerMap[REG_LED_RED]);
      analogWrite(LED_Green, 255-registerMap[REG_LED_GREEN]);
      analogWrite(LED_Blue,  255-registerMap[REG_LED_BLUE]);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      if(ledPhase == 0 && USB_HOST_CONNECTED) // USB -> blink green in pause color phase
      {
        analogWrite(LED_Red,   255);
        analogWrite(LED_Green, 230);
        analogWrite(LED_Blue,  255);
        digitalWrite(LED_BUILTIN, LOW);
      }
      else if(ledPhase == 1 && registerMap[REG_SHIELD]) // SHIELD contact -> blink yellow in pause color phase
      {
        analogWrite(LED_Red,   200);
        analogWrite(LED_Green, 230);
        analogWrite(LED_Blue,  255);
        digitalWrite(LED_BUILTIN, LOW);
      }
      else if(ledPhase == 2 && distance < distanceThreshold) // Distance -> blink blue in pause color phase
      {
        analogWrite(LED_Red,   255);
        analogWrite(LED_Green, 255);
        analogWrite(LED_Blue,  200);
        digitalWrite(LED_BUILTIN, LOW);
      }
      else if(ledPhase == 3 && force > forceThreshold) // Force -> blink purple in pause color phase
      {
        analogWrite(LED_Red,   200);
        analogWrite(LED_Green, 255);
        analogWrite(LED_Blue,  200);
        digitalWrite(LED_BUILTIN, LOW);
      }
      else
      {
        analogWrite(LED_Red,   255);
        analogWrite(LED_Green, 255);
        analogWrite(LED_Blue,  255);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    
  }

  //  end
  if(millis() - timer > 0)
  {
    timer = millis();
    framecounter++;
  }
}
//



//  callbacks
void callbackRead(uint8_t msgSize, const uint8_t* msg)
{
  if(msgSize >= 2 && msg[0] + msg[1] <= REG_SIZE + EEPROM_SIZE)
    NETWORK.sendRegister(NC_INST_REPLY, msg[1], &registerMap[msg[0]]);
}
void callbackWrite(uint8_t msgSize, const uint8_t* msg)
{
  if(msgSize < 2) return;
  switch(msg[0])
  {
    //  Status byte
    case REG_STATE:                                                     //  just mask to have access to eeprom locker
      registerMap[REG_STATE] &= ~STATUS_WRITE_MASK;
      registerMap[REG_STATE] |= msg[1]&STATUS_WRITE_MASK;
      break;


    //  Led
    case REG_LED_RED:   registerMap[REG_LED_RED] = msg[1];      break;  //  change led color
    case REG_LED_GREEN: registerMap[REG_LED_GREEN] = msg[1];    break;  //  idem
    case REG_LED_BLUE:  registerMap[REG_LED_BLUE] = msg[1];     break;  //  idem


    //  ID
    case REG_SIZE + EEPROM_ID:                                          //  change id dynamically and erase eeprom id if needed
      NETWORK.sendRegister(NC_INST_REPLY, 0, NULL); //ACK now because you can't after changing your id
      if(NETWORK.isPersonalId(msg[1]))
        break;
      
      registerMap[REG_SIZE + EEPROM_ID] = msg[1];
      NETWORK.setId(msg[1]);
      if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
        EEPROM.update(EEPROM_ID, msg[1]);
      break;


    //  Distance Threshold
    case REG_SIZE + EEPROM_DISTANCE_THSD_H:
      registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H] = msg[1];
      if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
        EEPROM.update(EEPROM_DISTANCE_THSD_H, msg[1]);
      if(msgSize >= 3)
      {
        registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = msg[2];
        if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
          EEPROM.update(EEPROM_DISTANCE_THSD_L, msg[2]);
      }
      distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
      break;
    case REG_SIZE + EEPROM_DISTANCE_THSD_L:
      registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = msg[1];
      if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
        EEPROM.update(EEPROM_DISTANCE_THSD_L, msg[1]);
      distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
      break;


    //  Force Threshold
    case REG_SIZE + EEPROM_FORCE_THSD_H:
      registerMap[REG_SIZE + EEPROM_FORCE_THSD_H] = msg[1];
      if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
        EEPROM.update(EEPROM_FORCE_THSD_H, msg[1]);
      if(msgSize >= 3)
      {
        registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = msg[2];
        if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
          EEPROM.update(EEPROM_FORCE_THSD_L, msg[2]);
      }
      forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];
      break;
    case REG_SIZE + EEPROM_FORCE_THSD_L:
      registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = msg[1];
      if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
        EEPROM.update(EEPROM_FORCE_THSD_L, msg[1]);
      forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];
      break;


    //  Error case: just send ACK
    default: break;
  }
  if(NETWORK.isPersonalId())
    NETWORK.sendRegister(NC_INST_REPLY, 0, NULL); //ACK
}
void callbackAction(uint8_t msgSize, const uint8_t* msg)
{
  framecounter = 0;
}
void callbackAutocalibrate(uint8_t msgSize, const uint8_t* msg)
{
  uint16_t f = 0;
  for(uint8_t i=0; i<8; i++)
    f += analogRead(FORCE);
  f >>= 3;
  registerMap[REG_SIZE + EEPROM_FORCE_THSD_H] = (f >> 8);
  registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = f;
}
//

