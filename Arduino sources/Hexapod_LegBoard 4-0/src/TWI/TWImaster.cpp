/*
 * TWImaster.cpp
 *
 * Created: 08/12/2018 18:10:40
 *  Author: Thibault-SED
 */ 


#include <avr/interrupt.h>

#include "TWImaster.h"



void TWI::initialize()
{
	TWBR = 12;
	TWDR = 0xFF;
	TWCR = (1<<TWEN);
}

bool TWI::write(const uint8_t& inst, const uint8_t& id, const uint8_t& msgSize, uint8_t* msg)
{
	while(TWCR&(1<<TWIE));
	buffer[0] = inst | (id<<1);
	if(inst == TWI_INST_WRITE)
	{
		for(uint8_t i=0; i<msgSize; i++)
			buffer[i+1] = msg[i];
	}
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // TWI enable, interrupt enable, clear flag & start condition
	validMessage = false;
	
	while(TWCR&(1<<TWIE));
	return validMessage;
}
bool TWI::read(const uint8_t& id, const uint8_t& msgSize, uint8_t* msg)
{
	while(TWCR&(1<<TWIE));
	if(validMessage)
	{
		for(uint8_t i=0; i<msgSize; i++)
			msg[i] = buffer[i];
	}
	return validMessage;
}



void TWI::interrupt()
{
	switch (TWSR)
	{
		case TWI_START:				// START has been transmitted  
		case TWI_REP_START:			// Repeated START has been transmitted
			bufferIndex = 0;
		case TWI_MTX_ADR_ACK:		// SLA+W has been transmitted and ACK received
		case TWI_MTX_DATA_ACK:		// Data byte has been transmitted and ACK received
			if (bufferIndex < messageSize)
			{
				TWDR = buffer[bufferIndex++];
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT); // TWI enable, interrupt enable & clear flag
			}
			else       				// Send STOP after last byte
			{
				validMessage = true;
				TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO);// TWI enable, interrupt enable & initiate a STOP condition
			}
			break;


		case TWI_MRX_DATA_ACK:		// Data byte has been received and ACK transmitted
			buffer[bufferIndex++] = TWDR;
		case TWI_MRX_ADR_ACK:		// SLA+R has been transmitted and ACK received
			if(bufferIndex < messageSize - 1)
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA); // TWI enable, interrupt enable, clear flag & send ACK
			else
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);	// TWI enable, interrupt enable, clear flag
			break;


		case TWI_MRX_DATA_NACK:		// Data byte has been received and NACK transmitted
			buffer[bufferIndex] = TWDR;
			validMessage = true;
			TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO);	// TWI enable, clear flag & initiate a STOP condition
			break;


		case TWI_ARB_LOST:
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA);// TWI enable, interrupt enable, clear flag & initiate re-start
			break;


		default:	// error or unexpected NACK received
			validMessage = false;
			TWCR = (1<<TWEN);	// TWI enable
			break;
	}
}


extern TWI I2C;
ISR(TWI_vect, ISR_NOBLOCK)
{
	I2C.interrupt();
}