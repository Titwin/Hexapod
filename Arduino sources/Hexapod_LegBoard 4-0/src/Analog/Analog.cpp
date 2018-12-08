/*
 * Analog.cpp
 *
 * Created: 08/12/2018 13:55:38
 *  Author: Thibault-SED
 */ 

 #include <avr/interrupt.h>

 #include "Analog.h"

 #define  ANALOG_STATES 5


void Analog::initialize()
{
	ADMUX = (0x01<<REFS0)|(FORCE);	// AVCC as reference, FORCE (ACD7) on MUX(4..0)
	ADCSRA = (1<<ADEN)|				// enable ADC
			 (1<<ADATE)|			// on trigger TIMER1_OVF
			 (1<<ADIE)|				// interrupt enable
			 (0x07<<ADPS0);			// set prescaler to 128 (125kHz -> 100µs conversion)
	DIDR0 = (1<<ADC7D);				// disable digital read on FORCE
	DIDR2 = (1<<ADC8D);				// disable digital read on V_BATT
	ADCSRB = 0x06<<ADTS0;				    // FORCE and trigger on TIMER1_OVF

	state = 0;
	force = 0xFFFF;
	temperature = 0xFFFF;
	VBatt = 0xFFFF;
}

uint16_t Analog::getForce() const { return force; }
uint16_t Analog::getVBatt() const { return VBatt; }
uint16_t Analog::getTemperature() const { return temperature; }

void inline Analog::interrupt()
{
	const uint16_t value = ADCL | (ADCH << 8);
	switch(state)
	{
		case 0:
			force = value;
			ADMUX = (0x01<<REFS0)|(V_BATT);
			break;
		case 1:
			VBatt = value;
			ADMUX = (0x03<<REFS0)|(TEMPERATURE);
			ADCSRB |= (1<<MUX5);
			break;
		case 3:
			temperature = value;
			ADMUX = (0x01<<REFS0)|(FORCE);
			ADCSRB &= ~(1<<MUX5);
			break;
		default: break;
	}
	state = (uint8_t)(state + 1)%ANALOG_STATES;
}

extern Analog ANALOG;
ISR(ADC_vect, ISR_NOBLOCK)
{
	ANALOG.interrupt();
}
