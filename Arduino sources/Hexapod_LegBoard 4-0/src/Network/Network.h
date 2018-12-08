/*
 * Network.h
 *
 * Created: 07/12/2018 17:32:13
 *  Author: Thibault-SED
 */ 


#ifndef NETWORK_H_
#define NETWORK_H_


#include <avr/io.h>


//  network constant defines
#define SERIAL_RX_BUFFER_SIZE	128

#define NC_BROADCAST_ID			0xFE
#define NC_START_BYTE			0xFE

#define NC_INST_PING            0x01
#define NC_INST_ACTION          0x05
#define NC_INST_READ            0x02
#define NC_INST_WRITE           0x03
#define NC_INST_REG_WRITE       0x04
#define NC_INST_RESET           0x06
#define NC_INST_REPLY           0x07
#define NC_INST_AUTOCALIBRATE   0x08


class Network
{
	public:
		//  Miscellaneous
		typedef void (*NetworkCallback)(const uint8_t& msgSize, const uint8_t* msg);
		//
	
		//  default
		Network();
		//

		//  initialization & update
		void update();
		void initialize(const uint8_t& startingId = 0x01);
		void parseChar(const uint8_t& c);
		//

		//  public functions
		bool assignCallback(const uint8_t& instructionType, NetworkCallback callback);
		void sendRegister(const uint8_t& instruction, const uint8_t& regSize, const uint8_t* reg);
		uint32_t getLastTimestamp();
		bool isPersonalId(const uint8_t& Id = 0xFF);
		void setId(const uint8_t& newid);
		//

		//  ISR vector interrupt
		void rx_complete_irq();
		//

	protected:
		//  protected attributes
		uint8_t id;
		uint8_t state;
		uint8_t currentMsg[20];
		uint8_t msgLength, msgIndex, crc, msgInst, msgId;
		uint32_t lastValidCharTimestamp;
		NetworkCallback callbackArray[7];

		volatile uint8_t buffer_head;
		volatile uint8_t buffer_tail;
		volatile uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
		//

		//  network management
		void resetState();
		void inline printHeader(const uint8_t& id, const uint8_t& msgSize, const uint8_t& start = NC_START_BYTE);
		uint8_t inline write(uint8_t c);
		uint8_t read();
		uint8_t available();
		//
};

#endif /* NETWORK_H_ */