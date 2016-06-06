#include <SharpIR.h>

#include "RaspberryPiCommand.h"
#include "UserCommand.h"

#define FPS 16
#define SPEED_LIMIT (4 * FPS)

MySCServo SERVO;
String message;

int MotorPosition[24] = {0,0,0,0, 0,0,601,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
int TargetPosition[24] = {0,0,0,0, 0,0,601,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
uint8_t MotorId[24] = {2,3,4,5, 6,7,8,9, 10,11,12,13, 14,15,16,17, 18,19,20,21, 22,23,24,25};
int SendedPosition[24];
int distance[3] = {0,0,0};
bool openUmbrela = false;

int debugLong = 0;

const PROGMEM int RecordedPos[]  = {
      551,  586,  470,  463,     529,  548,  601,  565,     512,  514,  534,  525,      563,  445,  588,  550,     458,  503,  493,  492,     514,  506,  470,  537,

      552,  587,  468,  464,     517,  747,  685,  557,     513,  515,  535,  521,      564,  740,  589,  548,     457,  502,  491,  491,     516,  764,  471,  536,
      381,  587,  471,  463,     434,  547,  682,  599,     715,  515,  532,  521,      398,  463,  587,  548,     436,  500,  457,  635,     632,  514,  482,  537,
      550,  874,  471,  464,     527,  547,  596,  566,     511,  819,  534,  523,      562,  448,  587,  548,     460,  733,  491,  494,     517,  506,  473,  536,
      571,  584,  489,  609,     600,  548,  609,  566,     306,  527,  531,  528,      598,  444,  586,  652,     613,  500,  457,  587,     323,  508,  473,  536
  };



void WDT_off()
{
  cli();
  asm("wdr");
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
  sei();
}
void WDT_on()
{
  cli();
  asm("wdr");
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE)|(1<<WDE);
  WDTCSR = (1<<WDE)|(0x02<<WDP0); // 125ms
  sei();
}
int get_IR (uint16_t value) {
        if (value < 16)  value = 16;
        return 2076.0 / (value - 11.0);
}


void setup()
{
  Serial2.begin(1000000); //init Serial baudrate (motor)
  Serial2.setTimeout(5);

  Serial1.begin(115200);  //init Serial baudrate
  Serial1.setTimeout(5);
  
  Serial.begin(250000);
  Serial.setTimeout(6000);
  
  SERVO.InitForSC();
  SERVO.WriteLimitTroque(0xfe,1024);
  pinMode(A2,INPUT);
  pinMode(A5,INPUT);
  pinMode(A3,INPUT);
  pinMode(7, OUTPUT);

  for(uint8_t i=0;i<24;i++)
  {
    int pos = SERVO.ReadPos(i+2);
    if(pos<0) continue;
    
    MotorPosition[i]  = pos;
    TargetPosition[i] = pos;
  }
  
  Serial.println("Hello friend !");
  WDT_on();
}





long lopper = 0;
int pos;
unsigned long start;

void loop()
{
  start = millis();
  parseUserCommand();
  
  while(Serial1.available())
  {
    int c = Serial1.read();
    if(c>=0) parseRaspberryPiCommand(c);
  }
  
  for(uint8_t i=0;i<24;i++)
  {
    if(i == 6) continue;
    //else if(i>=8) continue;
    
    pos = SERVO.ReadPos(i+2);
    if(pos >= 0) 
    {
      MotorPosition[i] = pos;
      
      if(TargetPosition[i] - MotorPosition[i] > SPEED_LIMIT) 
        SendedPosition[i] = MotorPosition[i] + SPEED_LIMIT;
        
      else if(TargetPosition[i] - MotorPosition[i] < -SPEED_LIMIT)
        SendedPosition[i] = MotorPosition[i] - SPEED_LIMIT;
        
      else
        SendedPosition[i] = TargetPosition[i];
    }
  }

  SERVO.SyncWritePos(MotorId,12,SendedPosition,FPS);
  SERVO.SyncWritePos(&MotorId[12],12,&SendedPosition[12],FPS);

  distance[0] = get_IR(analogRead(A2));
  distance[1] = get_IR(analogRead(A5));
  distance[2] = get_IR(analogRead(A3));

  //for(int i=0;i<3;i++)
  //  {Serial.print(distance[i]);Serial.print(' ');}
  //Serial.print('\n');
  
  if(openUmbrela)
  {
    digitalWrite(7,HIGH);
    delayMicroseconds(1000);
    digitalWrite(7,LOW);
  }
  else
  {
    digitalWrite(7,HIGH);
    delayMicroseconds(2000);
    digitalWrite(7,LOW);
  }

  //Serial.println(millis()-start);
  
  while(millis()-start < FPS);
  if(millis()-start > 10000) delayMicroseconds(5000);

  asm("wdr");
  lopper++;
  if(lopper%60 == 0)Serial.write('*');
  if(lopper%3600 == 0) Serial.write('\n');
}
















