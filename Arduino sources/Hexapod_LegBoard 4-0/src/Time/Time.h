/*
 * Time.h
 *
 * Created: 07/12/2018 18:48:56
 *  Author: Thibault-SED
 */ 


#ifndef TIME_H_
#define TIME_H_


class Time
{
	public:
		void initialize();
		void delay(uint32_t ms);

		volatile uint32_t millis;
};

extern Time TIME;

#endif /* TIME_H_ */