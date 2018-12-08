/*
 * Analog.h
 *
 * Created: 08/12/2018 13:55:25
 *  Author: Thibault-SED
 */ 


#ifndef ANALOG_H_
#define ANALOG_H_

#include "../Configuration.h"


class Analog
{
	public:
		void initialize();

		uint16_t getForce() const;
		uint16_t getVBatt() const;
		uint16_t getTemperature() const;

		void inline interrupt();

	protected:
		volatile uint16_t force;
		volatile uint16_t VBatt;
		volatile uint16_t temperature;

		volatile uint8_t state;
};

#endif /* ANALOG_H_ */