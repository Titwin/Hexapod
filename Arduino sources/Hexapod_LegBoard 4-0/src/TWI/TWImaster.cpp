/*
 * TWImaster.cpp
 *
 * Created: 08/12/2018 18:10:40
 *  Author: Thibault-SED
 */ 


#include <avr/interrupt.h>

#include "TWImaster.h"

#ifndef NULL
	#define NULL 0
#endif

#define TWCR_STOP  ((1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTO))
#define TWCR_REPLY ((1<<TWEN)|(1<<TWIE)|(1<<TWINT))
#define TWCR_START ((1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA))




void TWI::initialize()
{
	PORTD |= (1<<PORTD1)|(1<<PORTD0);		// SDA & SCL as INPUT_PULLUP
	TWBR = ((16000000 / 100000) - 16) / 2;	// 100kHz
	TWDR = 0xFF;							// init DATA
	TWCR = (1<<TWEN);						// enable module
	state = TWI_RDY;
	status = 0x00;
}

uint8_t TWI::write(const uint8_t& id, const uint8_t& reg, const uint8_t& msgSize, uint8_t const* msg)
{
	wait();

	state = TWI_MTX;
	buffer[0] = TWI_INST_WRITE | (id<<1);
	buffer[1] = reg;
	for(uint8_t i=0; i<msgSize; i++)
		buffer[i+2] = msg[i];
	messageSize = msgSize + 2;
	TWCR = TWCR_START;
	
	wait();
	return status;
}
uint8_t TWI::read(const uint8_t& id, const uint8_t& reg, const uint8_t& msgSize, uint8_t* const msg)
{
	wait();

	state = TWI_MTX;
	buffer[0] = TWI_INST_WRITE | (id<<1);
	buffer[1] = reg;
	messageSize = 2;
	TWCR = TWCR_START;

	wait();

	state = TWI_MRX;
	buffer[0] = TWI_INST_READ | (id<<1);
	messageSize = msgSize;
	TWCR = TWCR_START;
	
	wait();
	if(status == 0x00)
	{
		for(uint8_t i=0; i<msgSize; i++)
			msg[i] = buffer[i];
	}
	return status;
}
uint8_t TWI::ping(const uint8_t& id)
{
	wait();

	state = TWI_MTX;
	buffer[0] = TWI_INST_WRITE | (id<<1);
	messageSize = 1;
	TWCR = TWCR_START;

	wait();
	return (status == 0x00);
}
uint8_t TWI::scan(uint8_t& idSize, uint8_t* ids)
{
	idSize = 0;
	for(uint8_t i=3; i<127; i++)
	{
		if((i>>2) == 0x01) continue;
		if((i>>2) == 0x1E) continue;
		if((i>>2) == 0x1F) continue;

		if(ping(i))
		{
			if(ids)
				ids[idSize] = i;
			idSize++;
		}
	}
	return idSize;
}

void inline TWI::wait()
{
	while(state != TWI_RDY);
}



void TWI::interrupt()
{
	switch (TWSR)
	{
		case TWI_START:
		case TWI_REP_START:
			bufferIndex = 0;
			TWDR = buffer[bufferIndex];
			status = 0x00;
			TWCR = TWCR_REPLY;
			break;


		case TWI_MTX_ADR_ACK:
			bufferIndex++;
		case TWI_MTX_DATA_ACK:
			if (bufferIndex < messageSize)
			{
				TWDR = buffer[bufferIndex];
				bufferIndex++;
				TWCR = TWCR_REPLY;
			}
			else // last byte sent
			{
				status = 0x00;
				state = TWI_RDY;
				TWCR = TWCR_STOP;
			}
			break;


		case TWI_MRX_DATA_ACK:
			buffer[bufferIndex] = TWDR;
			bufferIndex++;
		case TWI_MRX_ADR_ACK:
			if(bufferIndex < messageSize-1)
				TWCR = TWCR_REPLY|(1<<TWEA);
			else TWCR = TWCR_REPLY;
			break;


		case TWI_MRX_DATA_NACK: // last byte received
			buffer[bufferIndex] = TWDR;
			status = 0x00;
			state = TWI_RDY;
			TWCR = TWCR_STOP;
			break;


		default:	// error or unexpected NACK received
			TWCR = TWCR_STOP;
			status = TWSR;
			state = TWI_RDY;
			break;
	}
}


extern TWI I2C;
ISR(TWI_vect)
{
	I2C.interrupt();
}