#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

#include "NetworkController.h"
#include "Configuration.h"
#include "Watchdog.h"

#define LED_BLINK_TIME 500  // in ms
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
  EEPROM.update(EEPROM_SLAVE_TYPE, LEGBOARD);
  EEPROM.update(EEPROM_VERSION, 0x01);
  
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
  
  //  LED setup
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN,   LOW);
  pinMode(LED_Red,   OUTPUT);   analogWrite(LED_Red,   200);
  pinMode(LED_Green, OUTPUT);   analogWrite(LED_Green, 200);
  pinMode(LED_Blue,  OUTPUT);   analogWrite(LED_Blue,  200);
  registerMap[REG_LED_RED] = 50;
  registerMap[REG_LED_GREEN] = 0;
  registerMap[REG_LED_BLUE] = 0;


  //  Shield contacts
  pinMode(SHIELD_A, INPUT_PULLUP);
  pinMode(SHIELD_B, INPUT_PULLUP);
  pinMode(SHIELD_C, INPUT_PULLUP);
  pinMode(SHIELD_D, INPUT_PULLUP);
  pinMode(SHIELD_E, INPUT_PULLUP);
  registerMap[REG_SHIELD] = 0x00;


  //  Force sensor
  pinMode(FORCE, INPUT);
  registerMap[REG_FORCE_H] = 0xFF;
  registerMap[REG_FORCE_L] = 0xFF;


  //  Serials connections
  if(registerMap[REG_SIZE + EEPROM_ID] >= 0xFE || registerMap[REG_SIZE + EEPROM_ID] == 0x00)
  {
    registerMap[REG_SIZE + EEPROM_ID] = 0x01;
    EEPROM.update(EEPROM_ID, 0x01);
  }
  NETWORK.initialize(registerMap[REG_SIZE + EEPROM_ID]);
  NETWORK.assignCallback(NC_INST_READ, &callbackRead);
  NETWORK.assignCallback(NC_INST_WRITE, &callbackWrite);
  Serial.begin(250000);


  //  distance sensor
  Wire.begin();
  delay(50);
  pinMode(VLX_RDY, INPUT);
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
  timer = millis();

  delay(1000);
  WDOG.on();
}
//


//  Program
unsigned long framecounter = 0;
void loop()
{
  //  begin && USB connection detect
  WDOG.reset();
  long start = micros();
  if(USB_HOST_CONNECTED) registerMap[REG_STATE] |= (1<<SERIAL_USB);
  else registerMap[REG_STATE] &= ~(1<<SERIAL_USB);
  if(framecounter > LED_BLINK_TIME) framecounter = 0;
  
  //  network
  NETWORK.update();

  //  Compute Thresholds
  distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
  forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) + registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];

  
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
    analogWrite(LED_Red,   255);
    analogWrite(LED_Green, 255);
    analogWrite(LED_Blue,  255);
    digitalWrite(LED_BUILTIN, LOW);
  }


  //  read distance
  if((registerMap[REG_STATE]&(1<<VLX_OK)) && digitalRead(VLX_RDY) == LOW)
  {
    uint16_t distance = distanceSensor.fastReadRange();
    registerMap[REG_DISTANCE_H] = distance>>8;
    registerMap[REG_DISTANCE_L] = distance&0xFF;

    if(distance < distanceThreshold)
      registerMap[REG_STATE] |= 1<<DISTANCE_THSD;
    else registerMap[REG_STATE] &= ~(1<<DISTANCE_THSD);
  }
  else if(!(registerMap[REG_STATE]&(1<<VLX_OK)) && distanceSensor.init())
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


  //  read force
  int16_t force = analogRead(FORCE);
  registerMap[REG_FORCE_H] = force>>8;
  registerMap[REG_FORCE_L] = force&0xFF;
  if(force > forceThreshold)
    registerMap[REG_STATE] |= 1<<FORCE_THSD;
  else registerMap[REG_STATE] &= ~(1<<FORCE_THSD);
  

  //  shield state changes
  registerMap[REG_SHIELD] = 0x00;
  if(digitalRead(SHIELD_A) == LOW) registerMap[REG_SHIELD] |= 1<<0;
  if(digitalRead(SHIELD_C) == LOW) registerMap[REG_SHIELD] |= 1<<1;
  if(digitalRead(SHIELD_B) == LOW) registerMap[REG_SHIELD] |= 1<<2;
  if(digitalRead(SHIELD_D) == LOW) registerMap[REG_SHIELD] |= 1<<3;
  if(digitalRead(SHIELD_E) == LOW) registerMap[REG_SHIELD] |= 1<<4;
  if(registerMap[REG_SHIELD]) registerMap[REG_STATE] |= 1<<SHIELD_CONTACT;
  else registerMap[REG_STATE] &= ~(1<<SHIELD_CONTACT);


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
      if(registerMap[REG_SIZE + EEPROM_ID] != msg[1])
        NETWORK.sendRegister(NC_INST_REPLY, 0, NULL); //ACK now because you can't after changing your id
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
//
