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


//	MAIN DEFINES
#define LED_BLINK_TIME 500  // in ms
#define LED_PHASE_COUNT 4
#define USB_HOST_CONNECTED (USBSTA&(1<<VBUS) && UDADDR&(1<<ADDEN))


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

	//	Main variables, sensors, ...
	uint8_t registerMap[REG_SIZE + EEPROM_SIZE];

	//	Variables alias
	uint16_t* const distanceThreshold = (uint16_t*)&registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H];
	uint16_t* const forceThreshold = (uint16_t*)&registerMap[REG_SIZE + EEPROM_FORCE_THSD_H];
	uint16_t* const forceOffset = (uint16_t*)&registerMap[REG_SIZE + EEPROM_FORCE_OFFSET_H];
	uint16_t* const distance = (uint16_t*)&registerMap[REG_DISTANCE_H];
	uint16_t* const force = (uint16_t*)&registerMap[REG_FORCE_H];
	uint16_t* const vbatt = (uint16_t*)&registerMap[REG_VBATT_H];
	uint16_t* const temperature = (uint16_t*)&registerMap[REG_TEMPERATURE_H];

	//	Timing variables
	uint32_t timer;
	uint16_t framecounter = 0;
	uint8_t ledPhase = 0;
//


/*	PROGRAM SETUP
 *  board initialization
 */
void inline setup()
{
	//  Type, version & time initialization
	eeprom_update_byte((uint8_t*)EEPROM_SLAVE_TYPE, LEGBOARD);
	eeprom_update_byte((uint8_t*)EEPROM_SOFT_VERSION, 0x03);

	//  LED, status bytes & EEPROM load
	{
		//  RGB LED setup
		PORTB |= (1<<PORTB7)|(1<<PORTB6)|(1<<PORTB5);	// RED, GREEN & BLUE LED off
		DDRB |= (1<<DDB7)|(1<<DDB6)|(1<<DDB5);			// RED, GREEN & BLUE as output
		registerMap[REG_LED_RED]   = 50;
		registerMap[REG_LED_GREEN] = 0;
		registerMap[REG_LED_BLUE]  = 0;

		//  status bytes & EEPROM load
		registerMap[REG_STATE] = (1<<EEPROM_LOCK);
		registerMap[REG_SIZE + EEPROM_ID] = eeprom_read_byte((uint8_t*)EEPROM_ID);
		registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_H] = eeprom_read_byte((uint8_t*)EEPROM_DISTANCE_THSD_H);
		registerMap[REG_SIZE + EEPROM_DISTANCE_THSD_L] = eeprom_read_byte((uint8_t*)EEPROM_DISTANCE_THSD_L);
		registerMap[REG_SIZE + EEPROM_FORCE_THSD_H]    = eeprom_read_byte((uint8_t*)EEPROM_FORCE_THSD_H);
		registerMap[REG_SIZE + EEPROM_FORCE_THSD_L]    = eeprom_read_byte((uint8_t*)EEPROM_FORCE_THSD_L);
		registerMap[REG_SIZE + EEPROM_SLAVE_TYPE]      = eeprom_read_byte((uint8_t*)EEPROM_SLAVE_TYPE);
		registerMap[REG_SIZE + EEPROM_SOFT_VERSION]    = eeprom_read_byte((uint8_t*)EEPROM_SOFT_VERSION);
	}
	//  Network, serials & Shield contacts
	{	
		//  Serials connections
		if(registerMap[REG_SIZE + EEPROM_ID] >= 0xFE || registerMap[REG_SIZE + EEPROM_ID] == 0x00)
		{
			registerMap[REG_SIZE + EEPROM_ID] = 0x01;
			eeprom_update_byte((uint8_t*)EEPROM_ID, 0x01);
		}
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

		//  ADC
		ANALOG.initialize();
		*force = 0xFFFF;
		*distance = 0xFFFF;

		registerMap[REG_STATE] &= ~(1<<VLX_OK);
	}
	
	//  start
	timer = TIME.millis;
	WDOG.on();
}



/*	PROGRAM LOOP
 *  the main loop ...
 */
void inline loop()
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

	//  shield state changes & compute Thresholds
	{
		registerMap[REG_SHIELD] = 0x00;
		if(!(PINF&(1<<PINF0))) registerMap[REG_SHIELD] |= 1<<4;	// center contact
		if(!(PINF&(1<<PINF1))) registerMap[REG_SHIELD] |= 1<<3;	// D contact
		if(!(PINF&(1<<PINF4))) registerMap[REG_SHIELD] |= 1<<0;	// A contact
		if(!(PINF&(1<<PINF5))) registerMap[REG_SHIELD] |= 1<<2;	// B contact
		if(!(PINF&(1<<PINF6))) registerMap[REG_SHIELD] |= 1<<1;	// C contact
	}

	//  Analog sensors & distance
	{ 
		*vbatt = ANALOG.getVBatt();
		*temperature = ANALOG.getTemperature();
		*force = ANALOG.getForce();

		if(*force > *forceThreshold)
			registerMap[REG_STATE] |= 1<<FORCE_THSD;
		else registerMap[REG_STATE] &= ~(1<<FORCE_THSD);
		
	}

	//  RGB LED color change
	{
		if(framecounter <= (uint16_t)(LED_BLINK_TIME/4)*((registerMap[REG_STATE]&0x03) + 1)) // turn light on
			writeLEDColor(registerMap[REG_LED_RED], registerMap[REG_LED_GREEN], registerMap[REG_LED_BLUE]);
		else
		{
			if(ledPhase == 0 && USB_HOST_CONNECTED)
				writeLEDColor(0, 25, 0);
			else if(ledPhase == 1 && registerMap[REG_SHIELD])
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
	}
}



/*	PROGRAM ENTRY
 *  we keep it simple in order to have an Arduino like program
 */
int main(void)
{
	asm("sei");
	TIME.initialize();
    setup();

    while (1)
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
}

//	calibrate sensors
void callbackAutocalibrate(const uint8_t& msgSize, const uint8_t* msg)
{
	
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
			NETWORK.sendRegister(NC_INST_REPLY, 0, NULL);					//ACK now because you can't after changing your id
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
