/*
 * TWImaster.h
 *
 * Created: 08/12/2018 18:10:25
 *  Author: Thibault-SED
 */ 


#ifndef TWIMASTER_H_
#define TWIMASTER_H_


// Miscellaneous
#define TWI_BUFFER_SIZE	64
#define TWI_INST_WRITE  0x00
#define TWI_INST_READ	0x01


// General TWI Master status codes
#define TWI_START                  0x08  // START has been transmitted
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition


class TWI
{
	public:
		void initialize();

		bool write(const uint8_t& inst, const uint8_t& id, const uint8_t& msgSize, uint8_t* msg);
		bool read(const uint8_t& id, const uint8_t& msgSize, uint8_t* msg);
		
		void interrupt();

	protected:
		volatile uint8_t bufferIndex;
		volatile uint8_t buffer[TWI_BUFFER_SIZE];

		volatile uint8_t messageSize;
		volatile bool validMessage;
};


#endif /* TWIMASTER_H_ */