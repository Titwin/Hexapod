#ifndef RASPBERRYPICOMMAND_H_INCLUDED
#define RASPBERRYPICOMMAND_H_INCLUDED

#include <MySCServo.h>

enum CommandCode {
  Ping = 0x01,
  
  SetMotorPosition = 0x02,
  GetMotorPosition = 0x03,
  EnableDisableTorque = 0x04,
  OpenUmbrella = 0x05
};


void parseRaspberryPiCommand(uint8_t c);

void PingCb();
void SetMotorPositionCb();
void GetMotorPositionCb();
void EnableDisableTorqueCb();
void OpenUmbrellaCb();

extern int debugLong;
extern MySCServo SERVO;
extern int MotorPosition[];
extern int TargetPosition[];
extern int distance[3];
extern bool openUmbrela;

#endif // RASPBERRYPICOMMAND_H_INCLUDED











