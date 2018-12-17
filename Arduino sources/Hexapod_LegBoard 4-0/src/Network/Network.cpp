/*
 * Network.cpp
 *
 * Created: 07/12/2018 17:32:25
 *  Author: Thibault-SED
 */

#include <avr/interrupt.h>

#include "Network.h"
#include "../../src/Time/Time.h"


#ifndef NULL
	#define NULL 0
#endif
 #define enableRX()  {UCSR1B |= (1<<RXEN1);  UCSR1B &=~(1<<TXEN1);  DDRD &= ~(1<<DDD2); PORTD |= (1<<PORTD2);}
 #define disableRX() {UCSR1B &=~(1<<RXEN1);  UCSR1B |= (1<<TXEN1);}



//  default
Network::Network() : buffer_head(0), buffer_tail(0)
{}
//

//  special
void Network::initialize(const uint8_t& startingId)
{
	UCSR1A = 0;				// no double speed
	UBRR1H = 0;				// 1M baud
	UBRR1L = 0;				// 1M baud
	UCSR1C = 0x03<<UCSZ10;	// 8bits, 1 stop, no parity
	enableRX();				// enable RX, disable TX
	UCSR1B |= 1<<RXCIE1;    // enable RX interrupt 
	UCSR1B &= ~(1<<UDRIE1); // disable TX interrupt

	id = startingId;
	lastValidCharTimestamp = TIME.millis;

	callbackArray[0] = NULL;  //  NC_INST_PING
	callbackArray[1] = NULL;  //  NC_INST_ACTION
	callbackArray[2] = NULL;  //  NC_INST_READ
	callbackArray[3] = NULL;  //  NC_INST_WRITE
	callbackArray[4] = NULL;  //  NC_INST_ACTION
	callbackArray[5] = NULL;  //  NC_INST_RESET
	callbackArray[6] = NULL;  //  NC_INST_AUTOCALIBRATE
}
//


 //  network management
void Network::update()
{
	while(available())
		parseChar(read());
}
void Network::resetState()
{
	state = 0;
	msgIndex = 0;
	msgLength = 0;
	msgInst = 0;
	msgId = 0;
	crc = 0;
}
void inline Network::printHeader(const uint8_t& id, const uint8_t& msgSize, const uint8_t& start)
{
	 write(start);
	 write(start);
	 write(id);
	 write(msgSize);
}
uint8_t inline Network::write(uint8_t c)
{
	UDR1 = c;
	while(!(UCSR1A & (1<<UDRE1)));
	return 1;
}
uint8_t Network::read()
{
	uint8_t c = buffer[buffer_tail];
	buffer_tail = (uint8_t)(buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
	return c;
}
uint8_t Network::available()
{
	return ((uint8_t)(SERIAL_RX_BUFFER_SIZE + buffer_head - buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}
void Network::parseChar(const uint8_t& c)
{
	lastValidCharTimestamp = TIME.millis;
	switch(state)
	{
		// parsing message header
		case 0: case 1:
			if(c == NC_START_BYTE) state++;
			else state = 0;
			break;

		case 2:
			if(c == id || c == NC_BROADCAST_ID) state++;
			else state = 0;
			crc = c;
			msgId = c;
			break;

		case 3:
			crc += c;
			msgLength = c;
			state++;
			break;
		 
		case 4:
			crc += c;
			msgInst = c;
			if(msgInst == NC_INST_PING || msgInst == NC_INST_ACTION || msgInst == NC_INST_RESET || msgInst == NC_INST_AUTOCALIBRATE)
				state = 6;
			else if(msgInst == NC_INST_REPLY) // ignore echo & others messages
				resetState();
			else state++;
			break;

		// save incoming message
		case 5:
			crc += c;
			currentMsg[msgIndex] = c;
			msgIndex++;
			if(msgIndex+2 >= msgLength)
				state = 6;
			break;

		// check crc and respond
		case 6:
			if(c == ((~crc)&0xFF))
			{
				switch(msgInst)
				{
					case NC_INST_PING:
						if(callbackArray[0])(*callbackArray[0])(0, NULL);
						if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
						break;

					case NC_INST_ACTION:
						if(callbackArray[1]) (*callbackArray[1])(0, NULL);
						break;
				 
					case NC_INST_READ:
						if(msgId == id && callbackArray[2]) (*callbackArray[2])(msgLength-2, currentMsg);
						break;
				 
					case NC_INST_WRITE:
						if(callbackArray[3]) (*callbackArray[3])(msgLength-2, currentMsg);
						break;
				 
					case NC_INST_SYNC_WRITE:
						if(callbackArray[4]) (*callbackArray[4])(msgLength-2, currentMsg);
						if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
						break;

					case NC_INST_RESET:
						if(callbackArray[5])(*callbackArray[5])(0, NULL);
						if(msgId == id) sendRegister(NC_INST_REPLY, 0, NULL);
						while(1);
						break;

					case NC_INST_AUTOCALIBRATE:
						if(callbackArray[6])(*callbackArray[6])(0, NULL);
						break;
				 
					default: break;
				}
			}
			resetState();
			break;
		 
		default:
			resetState();
			break;
	}
}
//

//  public functions
bool Network::assignCallback(const uint8_t& instructionType, NetworkCallback callback)
{
	switch(instructionType)
	{
		case NC_INST_PING:          callbackArray[0] = callback; break;
		case NC_INST_ACTION:        callbackArray[1] = callback; break;
		case NC_INST_READ:          callbackArray[2] = callback; break;
		case NC_INST_WRITE:         callbackArray[3] = callback; break;
		case NC_INST_SYNC_WRITE:    callbackArray[4] = callback; break;
		case NC_INST_RESET:         callbackArray[5] = callback; break;
		case NC_INST_AUTOCALIBRATE: callbackArray[6] = callback; break;
		default: return false;
	}
	return true;
}
void Network::sendRegister(const uint8_t& instruction, const uint8_t& regSize, const uint8_t* reg)
{
	const uint8_t messageLength = 2 + regSize;
	uint8_t crc = id + messageLength + instruction;

	disableRX()
	printHeader(id, messageLength);
	write(instruction);
	for(uint8_t i=0; i<regSize; i++)
	{
		write(reg[i]);
		crc += reg[i];
	}
	write((~(crc))&0xFF);
	enableRX()
}
uint32_t Network::getLastTimestamp(){return lastValidCharTimestamp;}
bool Network::isPersonalId(const uint8_t& Id){return (Id==0xFF ? msgId==id : Id==id);};
void Network::setId(const uint8_t& newid){id = newid;}
void Network::clear() { buffer_head = 0; buffer_tail = 0; }
//

//  ISR vector interrupt
void Network::rx_complete_irq()
{
	const uint8_t c = UDR1;
	const uint8_t i = (uint8_t)(buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
	if (i != buffer_tail)
	{
		buffer[buffer_head] = c;
		buffer_head = i;
	}
}
extern Network NETWORK;
ISR(USART1_RX_vect)
{
	NETWORK.rx_complete_irq();
}
//
