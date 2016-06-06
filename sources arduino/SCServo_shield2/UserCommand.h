#ifndef USERCOMMAND_H_INCLUDED
#define USERCOMMAND_H_INCLUDED

#include <MySCServo.h>


int askUserFor(String s);
void parseUserCommand();


extern const PROGMEM int RecordedPos[];
extern MySCServo SERVO;


#endif // USERCOMMAND_H_INCLUDED
