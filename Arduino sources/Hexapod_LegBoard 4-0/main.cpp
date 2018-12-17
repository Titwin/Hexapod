/*
 * Hexapod_LegBoard.cpp
 *
 * Created: 07/12/2018 13:14:22
 * Author : Thibault-SED
 */ 


//	INCLUDES
#include <avr/io.h>
#include <avr/eeprom.h>

#include "src/Configuration.h"
#include "src/Time/Time.h"
#include "src/Watchdog/Watchdog.h"
#include "src/Network/Network.h"
#include "src/Analog/Analog.h"
#include "src/TWI/TWImaster.h"
#include "src/VL35L0X/VL53L0X.h"


//	MAIN DEFINES
#define LED_BLINK_TIME 500			// in ms
#define LED_ERROR_STATUS_TIME 3000	// in ms
#define LED_PHASE_COUNT 4
#define CALIBRATION_SAMPLES_POWER 4
//#define USB_HOST_CONNECTED (USBSTA&(1<<VBUS) && UDADDR&(1<<ADDEN))

#define min(a,b) (a<b ? a : b)


//	MAIN PROTOTYPES
void callbackRead(const uint8_t& msgSize, const uint8_t* msg);
void callbackWrite(const uint8_t& msgSize, const uint8_t* msg);
void callbackAction(const uint8_t& msgSize, const uint8_t* msg);
void callbackAutocalibrate(const uint8_t& msgSize, const uint8_t* msg);

void writeLEDColor(const uint8_t& r, const uint8_t& g, const uint8_t& b);



//	VARIABLES
	//	Class, Managers
	Watchdog WDOG(WDTO_60MS);
	Network NETWORK;
	Time TIME;
	Analog ANALOG;
	TWI I2C;
	VL53L0X VLX;

	//	Main variables, sensors, ...
	uint8_t registerMap[REG_SIZE + EEPROM_SIZE];

	//	Variables
	uint16_t distanceThreshold;
	uint16_t forceThreshold;

	uint16_t distance;
	uint16_t force;
	uint16_t vbatt;
	uint16_t temperature;


	//	Timing variables
	uint32_t timer, start;
	uint16_t framecounter;
	uint8_t ledPhase;
	
	//	Booting status & errors
	bool bootError = false;
	uint8_t boot_r, boot_g, boot_b;
//


/*	PROGRAM SETUP
 *  board initialization
 */
void inline setup()
{
	//  Type, version & time initialization
	eeprom_update_byte((uint8_t*)EEPROM_SLAVE_TYPE, LEGBOARD);
	eeprom_update_byte((uint8_t*)EEPROM_SOFT_VERSION, 0x03);
	start = TIME.millis;

	//  LED, status bytes & EEPROM load
	{
		//  RGB LED setup
		PORTB |= (1<<PORTB7)|(1<<PORTB6)|(1<<PORTB5);	// RED, GREEN & BLUE LED off
		DDRB |= (1<<DDB7)|(1<<DDB6)|(1<<DDB5);			// RED, GREEN & BLUE as output
		registerMap[REG_LED_RED]   = 50;
		registerMap[REG_LED_GREEN] = 0;
		registerMap[REG_LED_BLUE]  = 0;
		writeLEDColor(20, 20, 20);

		//  status bytes
		registerMap[REG_STATE] = (1<<EEPROM_LOCK);

		//	EEPROM load
		registerMap[REG_SIZE + EEPROM_ID]           = eeprom_read_byte((uint8_t*)EEPROM_ID);
		registerMap[REG_SIZE + EEPROM_SLAVE_TYPE]   = eeprom_read_byte((uint8_t*)EEPROM_SLAVE_TYPE);
		registerMap[REG_SIZE + EEPROM_SOFT_VERSION] = eeprom_read_byte((uint8_t*)EEPROM_SOFT_VERSION);

		registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]   = eeprom_read_byte((uint8_t*)EEPROM_DISTANCE_THSD_H);
		registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L]   = eeprom_read_byte((uint8_t*)EEPROM_DISTANCE_THSD_L);
		registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]   = eeprom_read_byte((uint8_t*)EEPROM_FORCE_THSD_H);
		registerMap[REG_SIZE + EEPROM_FORCE_THSD_L]   = eeprom_read_byte((uint8_t*)EEPROM_FORCE_THSD_L);
	}
	//  Network, serials & Shield contacts
	{
		//  Serials connections
		if(registerMap[REG_SIZE + EEPROM_ID] >= NC_BROADCAST_ID || registerMap[REG_SIZE + EEPROM_ID] == 0x00)
			registerMap[REG_SIZE + EEPROM_ID] = 0x01;
		NETWORK.initialize(registerMap[REG_SIZE + EEPROM_ID]);
		NETWORK.assignCallback(NC_INST_READ, &callbackRead);
		NETWORK.assignCallback(NC_INST_WRITE, &callbackWrite);
		NETWORK.assignCallback(NC_INST_ACTION, &callbackAction);
		NETWORK.assignCallback(NC_INST_AUTOCALIBRATE, &callbackAutocalibrate);


		//  Shield contacts
		PORTF |= (1<<PORTF0)|(1<<PORTF1)|(1<<PORTF4)|(1<<PORTF5)|(1<<PORTF6); // Shield A, B, C, D & E as INPUT_PULLUP
		registerMap[REG_SHIELD] = 0x00;
	}
	//  ADC & distance sensors
	{
		//	I2C & distance sensor
		I2C.initialize();
		TIME.delay(100);

		if(MCUSR&(1<<PORF))
		{
			switch(VLX.init())
			{
				case 0:	// no error
					TIME.delay(100);
					//VLX.setSignalRateLimit(0.1);
					VLX.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
					VLX.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
					TIME.delay(100);
					VLX.startContinuous();
					registerMap[REG_STATE] |= (1<<VLX_OK);
					break;

				case 1:
					bootError = true;
					boot_r = 0; boot_g = 20; boot_b = 0;
					registerMap[REG_STATE] &= ~(1<<VLX_OK);
					break;
				case 2:
					bootError = true;
					boot_r = 0; boot_g = 0; boot_b = 20;
					registerMap[REG_STATE] &= ~(1<<VLX_OK);
					break;
				case 3:
					bootError = true;
					boot_r = 20; boot_g = 0; boot_b = 20;
					registerMap[REG_STATE] &= ~(1<<VLX_OK);
					break;
				case 4:
					bootError = true;
					boot_r = 20; boot_g = 20; boot_b = 0;
					registerMap[REG_STATE] &= ~(1<<VLX_OK);
					break;

				default:
					bootError = true;
					boot_r = 0; boot_g = 20; boot_b = 20;
					registerMap[REG_STATE] &= ~(1<<VLX_OK);
					break;
			}
		}
		else
		{
			registerMap[REG_STATE] |= (1<<VLX_OK);
		}
		
		//  ADC
		ANALOG.initialize();
		force = 0x7FFF;
		distance = 0x7FFF;
		vbatt = 0x7FFF;
		temperature = 0x7FFF;

		registerMap[REG_FORCE_H] = force>>8;			 registerMap[REG_FORCE_L] = force&0xFF;
		registerMap[REG_DISTANCE_H] = distance>>8;		 registerMap[REG_DISTANCE_L] = distance&0xFF;
		registerMap[REG_VBATT_H] = vbatt>>8;			 registerMap[REG_VBATT_L] = vbatt&0xFF;
		registerMap[REG_TEMPERATURE_H] = temperature>>8; registerMap[REG_TEMPERATURE_L] = temperature&0xFF;
	}
	
	//  start
	timer = TIME.millis;
	framecounter = 0;
	ledPhase = 0;
	WDOG.on();
}



/*	PROGRAM LOOP
 *  the main loop ...
 */

void inline loop()
{
	//  begin && USB connection detect
	WDOG.reset();
	/*if(USB_HOST_CONNECTED) registerMap[REG_STATE] |= (1<<SERIAL_USB);
	else registerMap[REG_STATE] &= ~(1<<SERIAL_USB);*/
	if(framecounter > LED_BLINK_TIME)
	{
		framecounter = 0;
		ledPhase = (ledPhase +1)%LED_PHASE_COUNT;
	}

	//  network
	NETWORK.update();

	//  shield state changes & compute Thresholds
	{
		registerMap[REG_SHIELD] = 0x00;
		if(!(PINF&(1<<PINF0))) registerMap[REG_SHIELD] |= 1<<4;	// center contact
		if(!(PINF&(1<<PINF1))) registerMap[REG_SHIELD] |= 1<<3;	// D contact
		if(!(PINF&(1<<PINF4))) registerMap[REG_SHIELD] |= 1<<0;	// A contact
		if(!(PINF&(1<<PINF5))) registerMap[REG_SHIELD] |= 1<<2;	// B contact
		if(!(PINF&(1<<PINF6))) registerMap[REG_SHIELD] |= 1<<1;	// C contact

		if(registerMap[REG_SHIELD])
			registerMap[REG_STATE] |= (1<<SHIELD_CONTACT);
		else registerMap[REG_STATE] &= ~(1<<SHIELD_CONTACT);

		distanceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]<<8) | registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L];
		forceThreshold = ((uint16_t)registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]<<8) | registerMap[REG_SIZE + EEPROM_FORCE_THSD_L];
	}

	//  Analog sensors & distance
	{
		//	get analog sensors values
		vbatt = ANALOG.getVBatt();
		temperature = ANALOG.getTemperature();
		force = ANALOG.getForce();

		if(registerMap[REG_STATE]&(1<<VLX_OK) && !(PINB&(1<<PINB4)))
			distance = VLX.fastReadRange();

		//	update state register
		if(force > forceThreshold)
			registerMap[REG_STATE] |= 1<<FORCE_THSD;
		else registerMap[REG_STATE] &= ~(1<<FORCE_THSD);
		
		if(distance < distanceThreshold)
			registerMap[REG_STATE] |= 1<<DISTANCE_THSD;
		else registerMap[REG_STATE] &= ~(1<<DISTANCE_THSD);

		//	unpack data into map
		registerMap[REG_FORCE_H] = force>>8;			 registerMap[REG_FORCE_L] = force&0xFF;
		registerMap[REG_DISTANCE_H] = distance>>8;		 registerMap[REG_DISTANCE_L] = distance&0xFF;
		registerMap[REG_VBATT_H] = vbatt>>8;			 registerMap[REG_VBATT_L] = vbatt&0xFF;
		registerMap[REG_TEMPERATURE_H] = temperature>>8; registerMap[REG_TEMPERATURE_L] = temperature&0xFF;
	}

	//  RGB LED color change
	{
		if(bootError && timer - start < LED_ERROR_STATUS_TIME)
			writeLEDColor(boot_r, boot_g, boot_b);
		else if(framecounter <= (uint16_t)(LED_BLINK_TIME/4)*((registerMap[REG_STATE]&0x03) + 1)) // turn light on
			writeLEDColor(registerMap[REG_LED_RED], registerMap[REG_LED_GREEN], registerMap[REG_LED_BLUE]);
		else
		{
			/*if(ledPhase == 0 && false)
				writeLEDColor(0, 25, 0);
			else */if(ledPhase == 1 && registerMap[REG_SHIELD])
				writeLEDColor(55, 25, 0);
			else if(ledPhase == 2 && (registerMap[REG_STATE]&(1<<DISTANCE_THSD)))
				writeLEDColor(0, 0, 55);
			else if(ledPhase == 3 && (registerMap[REG_STATE]&(1<<FORCE_THSD)))
				writeLEDColor(55, 0, 55);
			else
				writeLEDColor(0, 0, 0);
		}
	}

	//	end
	if(TIME.millis > timer)
	{
		framecounter++;
		timer = TIME.millis;
		if(bootError && timer - start > LED_ERROR_STATUS_TIME)
			bootError = false;
	}
}



/*	PROGRAM ENTRY
 *  we keep it simple in order to have an Arduino like program
 */
int main(void)
{
	WDOG.off();			//	prevent booting watchdog reset
	if(MCUSR&(1<<WDRF))
		registerMap[REG_WDR_COUNT]++;
	else registerMap[REG_WDR_COUNT] = 0;

	TIME.initialize();	//	initialize static clock counter
    setup();
	MCUSR &= ~(0x1F);	//	clear reset register status

    while(1)
		loop();
}



/*	USEFULL FUNCTIONS
 *
 */
void writeLEDColor(const uint8_t& r, const uint8_t& g, const uint8_t& b)
{
	uint16_t red = (uint16_t)r * 251;   red >>= 2;	// = *62.75 to be in range of [0; 16000]
	uint16_t green = (uint16_t)g * 251; green >>= 2;
	uint16_t blue = (uint16_t)b * 251;  blue >>= 2;

	OCR1A = blue;
	OCR1B = green;
	OCR1C = red;
}


/*	NETWORK CALLBACKS
 */
//	read slave registers
void callbackRead(const uint8_t& msgSize, const uint8_t* msg)
{
	if(msgSize >= 2 && msg[0] + msg[1] <= REG_SIZE + EEPROM_SIZE)
		NETWORK.sendRegister(NC_INST_REPLY, msg[1], &registerMap[msg[0]]);
}

//	synchronize led blinks
void callbackAction(const uint8_t& msgSize, const uint8_t* msg)
{
	framecounter = 0;
	NETWORK.clear();
}

//	calibrate sensors
void callbackAutocalibrate(const uint8_t& msgSize, const uint8_t* msg)
{
	//	ACK & start
	uint8_t calibrationSamples = 0;
	uint16_t thsd = 0;
	
	//	getting samples
	while (calibrationSamples<(1<<CALIBRATION_SAMPLES_POWER))
	{
		if(ANALOG.getUpdates()&Analog::forceUpdate)
		{
			thsd += ANALOG.getForce();
			calibrationSamples++;
		}
		WDOG.reset();
	}

	//	force thresholds
	thsd = (thsd>>CALIBRATION_SAMPLES_POWER) + 20;
	registerMap[REG_SIZE + EEPROM_FORCE_THSD_H] = thsd>>8;
	registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = thsd&0xFF;
	eeprom_update_byte((uint8_t*)EEPROM_FORCE_THSD_H, registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]);
	eeprom_update_byte((uint8_t*)EEPROM_FORCE_THSD_L, registerMap[REG_SIZE + EEPROM_FORCE_THSD_L]);

	//	distance thresholds
	registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H] = 0;
	registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = 60;
	eeprom_update_byte((uint8_t*)EEPROM_DISTANCE_THSD_H, registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H]);
	eeprom_update_byte((uint8_t*)EEPROM_DISTANCE_THSD_L, registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L]);
	
	//	end
	if(NETWORK.isPersonalId())
		NETWORK.sendRegister(NC_INST_REPLY, 0, NULL);
}

//	change slaves registers values
void callbackWrite(const uint8_t& msgSize, const uint8_t* msg)
{
	if(msgSize < 2) return;
	switch(msg[0])
	{
		//  Status byte
		case REG_STATE:   //  just mask to have access to EEPROM locker
			registerMap[REG_STATE] &= ~STATUS_WRITE_MASK;
			registerMap[REG_STATE] |= msg[1]&STATUS_WRITE_MASK;
			break;


		//  Led
		case REG_LED_RED:   registerMap[REG_LED_RED] = msg[1];      break;
		case REG_LED_GREEN: registerMap[REG_LED_GREEN] = msg[1];    break;
		case REG_LED_BLUE:  registerMap[REG_LED_BLUE] = msg[1];     break;


		//  ID
		case REG_SIZE + EEPROM_ID:
			if(NETWORK.isPersonalId(msg[1]) || msg[1] == NC_BROADCAST_ID)
				break;
			NETWORK.sendRegister(NC_INST_REPLY, 0, NULL);  //ACK now because you can't after changing your id
			registerMap[REG_SIZE + EEPROM_ID] = msg[1];
			NETWORK.setId(msg[1]);
			if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
				eeprom_update_byte((uint8_t*)EEPROM_ID, msg[1]);
			break;


		//  Distance Threshold
		case REG_SIZE + EEPROM_DISTANCE_THSD_H:
			registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H] = msg[1];
			if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
				eeprom_update_byte((uint8_t*)EEPROM_DISTANCE_THSD_H, msg[1]);
			if(msgSize >= 3)
			{
				registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = msg[2];
				if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
					eeprom_update_byte((uint8_t*)EEPROM_DISTANCE_THSD_L, msg[2]);
			}
			break;
		case REG_SIZE + EEPROM_DISTANCE_THSD_L:
			registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = msg[1];
			if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
				eeprom_update_byte((uint8_t*)EEPROM_DISTANCE_THSD_L, msg[1]);
			break;


		//  Force Threshold
		case REG_SIZE + EEPROM_FORCE_THSD_H:
			registerMap[REG_SIZE + EEPROM_FORCE_THSD_H] = msg[1];
			if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
				eeprom_update_byte((uint8_t*)EEPROM_FORCE_THSD_H, msg[1]);
			if(msgSize >= 3)
			{
				registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = msg[2];
				if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
					eeprom_update_byte((uint8_t*)EEPROM_FORCE_THSD_L, msg[2]);
			}
			break;
		case REG_SIZE + EEPROM_FORCE_THSD_L:
			registerMap[REG_SIZE + EEPROM_FORCE_THSD_L] = msg[1];
			if(!(registerMap[REG_STATE]&(1<<EEPROM_LOCK)))
				eeprom_update_byte((uint8_t*)EEPROM_FORCE_THSD_L, msg[1]);
			break;


		//  Error case: just send ACK
		default: break;
	}
	if(NETWORK.isPersonalId())
		NETWORK.sendRegister(NC_INST_REPLY, 0, NULL); //ACK
}
