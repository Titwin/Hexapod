#ifndef CONFIGURATION_H
#define CONFIGURATION_H 

#include "Arduino.h"

#define VERBOSE 1
#define Network Serial1
#define enableRX()  {UCSR1B |= (1<<RXEN1);  UCSR1B &=~(1<<TXEN1);  DDRD &= ~(1<<DDD2); PORTD |= (1<<PORTD2);}
#define disableRX() {UCSR1B &=~(1<<RXEN1);  UCSR1B |= (1<<TXEN1);}

#define printf(args) (Network.write(args))
#define flush() (Network.flush())
#define scanf(buf,len) (Network.readBytes(buf,len))
#define printHeader(start,id,msgSize) {printf(start); printf(start); printf(id); printf(msgSize);}

#endif //CONFIGURATION_H
