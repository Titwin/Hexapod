/*
 * Time.cpp
 *
 * Created: 07/12/2018 18:53:31
 *  Author: Thibault-SED
 */ 

#include <avr/interrupt.h>

#include "Time.h"


 void Time::initialize()
 {
	millis = 0;
	TCCR1A = (0x03<<COM1A0)|		  // output high on compare match (led off)
			 (0x03<<COM1B0)|		  // output high on compare match (led off)
			 (0x03<<COM1C0)|		  // output high on compare match (led off)
			 (1<<WGM11)|(0<<WGM10);	  // fast PWM clear on ICR1
	ICR1 = 16000;					  // overflow each 1ms
	TIMSK1 = 1<<TOIE1;				  // overflow interrupt enable
	TCCR1B = (0x03<<WGM12)|(1<<CS10); // fast PWM clear on ICR1, no prescaler, timer on
 }
 void Time::delay(uint32_t ms)
 {
	const uint32_t t = millis;
	while(millis - t < ms);
 }

 
 ISR(TIMER1_OVF_vect)
 {
	TIME.millis++;
 }
