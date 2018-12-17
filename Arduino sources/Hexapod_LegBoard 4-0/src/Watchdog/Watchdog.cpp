/*
 * Watchdog.cpp
 *
 * Created: 07/12/2018 13:58:08
 *  Author: Thibault-SED
 */ 

#include "Watchdog.h"

Watchdog::Watchdog(uint8_t resetTime)
{
	 prescaler = resetTime;  // set the prescaler value
	 off();                  // turn of the watchdog to avoid unexpected and weird problem due to multiple reset at initialization
}

void Watchdog::off()
{
	asm("cli");
	reset();
	MCUSR &= ~(1<<WDRF);
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = 0x00;
	asm("sei");
}

void Watchdog::on()
{
	asm("cli");
	reset();
	MCUSR &= ~(1<<WDRF);
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = (1<<WDE)|(prescaler<<WDP0);
	asm("sei");
}

void Watchdog::setPrescaler(uint8_t resetTime)
{
	if(resetTime != prescaler)    // prevent to useless function call by checking the new prescaler value first
	{
		prescaler = resetTime;      // change prescaler value
		on();                       // turn on the watchdog
	}
}