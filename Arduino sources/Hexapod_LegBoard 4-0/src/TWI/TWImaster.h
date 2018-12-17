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
#define TWI_RDY                    0x00  // No relevant state information available; TWINT = “0”
#define TWI_MTX		               0x01
#define TWI_MRX		               0x02


class TWI
{
	public:
		void initialize();

		uint8_t write(const uint8_t& id, const uint8_t& reg, const uint8_t& msgSize, uint8_t const* msg);
		uint8_t read(const uint8_t& id, const uint8_t& reg, const uint8_t& msgSize, uint8_t* const msg);
		uint8_t ping(const uint8_t& id);
		uint8_t scan(uint8_t& idSize, uint8_t* ids);
		
		void interrupt();

	protected:
		volatile uint8_t bufferIndex;
		volatile uint8_t buffer[TWI_BUFFER_SIZE];

		volatile uint8_t messageSize;
		volatile uint8_t status;
		volatile uint8_t state;

		void inline wait();
};


#endif /* TWIMASTER_H_ */