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
		enum Updates
		{
			forceUpdate = 0x01,
			voltageUpdate = 0x02,
			temperatureUpdate = 0x04
		};

		void initialize();

		uint16_t getForce();
		uint16_t getVBatt();
		uint16_t getTemperature();
		uint8_t getUpdates();

		void inline interrupt();

	protected:
		volatile uint16_t force;
		volatile uint16_t VBatt;
		volatile uint16_t temperature;

		volatile uint8_t state;
		volatile uint8_t updates;
};

#endif /* ANALOG_H_ */