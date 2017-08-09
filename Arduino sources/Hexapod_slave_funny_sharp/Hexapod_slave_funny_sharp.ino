#include "Arduino.h"
#include "Configuration.h"
#include "NetworkController.h"
#include <Servo.h>


//  Global variable initialization
NetworkController NETWORK;
Servo SERVO_1;

bool running = false;
bool end = false;
unsigned long startTime;

uint8_t configuration;
int d1, d2;
//


//  Initialization
void setup()
{
  pinMode(PIN_START, INPUT_PULLUP);
  
  pinMode(PIN_LED_RED, OUTPUT);     digitalWrite(PIN_LED_RED, HIGH);
  pinMode(PIN_LED_GREEN, OUTPUT);   digitalWrite(PIN_LED_GREEN, HIGH);
  pinMode(PIN_LED_YELLOW, OUTPUT);  digitalWrite(PIN_LED_YELLOW, HIGH);

  pinMode(SWITCH_COM, INPUT_PULLUP);
  pinMode(SWITCH_A, OUTPUT);        digitalWrite(SWITCH_A, LOW);
  pinMode(SWITCH_B, OUTPUT);        digitalWrite(SWITCH_B, HIGH);
  
  NETWORK.initialize();
  SERVO_1.attach(PIN_SERVO_1);
  configuration = 0x00; // default team = blue
}
//


//  Program
int get_IR (uint16_t value);
unsigned long lastSonarPingTime = 0;
void loop()
{
  // start / reset detection
  if(digitalRead(PIN_START) == LOW && !running) //start
  {
    running = true;
    end = false;
    startTime = millis();
    SERVO_1.write(90);
    
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    configuration |= 1<<CONFIG_START;
  }
  else if(digitalRead(PIN_START) == HIGH && running) // reset
  {
    running = false;
    end = false;
    
    digitalWrite(PIN_LED_RED, HIGH);
    configuration &= ~((1<<CONFIG_START)|(1<<CONFIG_FUNNY));
  }

  // team display
  if(digitalRead(PIN_START) == HIGH)
  {
    if(digitalRead(SWITCH_COM) == HIGH)
    {
       digitalWrite(PIN_LED_GREEN, LOW); // blue team
       digitalWrite(PIN_LED_YELLOW, HIGH);
       configuration &= ~(1<<CONFIG_TEAM);
    }
    else
    {
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_YELLOW, LOW);
      configuration |= 1<<CONFIG_TEAM; // yellow team
    }
  }

  // Sonar ping
  d1 = get_IR(analogRead(SHARP_1));
  d2 = get_IR(analogRead(SHARP_2));
  if(d1 > 0 && d1 < DISTANCE_LED_TRESHOLD)
  {
    digitalWrite(PIN_LED_RED, LOW);
    configuration |= 1<< CONFIG_OBSTACLE;
  }
  else if(d2 > 0 && d2 < DISTANCE_LED_TRESHOLD)
  {
    digitalWrite(PIN_LED_RED, LOW);
    configuration |= 1<< CONFIG_OBSTACLE;
  }
  else
  {
    digitalWrite(PIN_LED_RED, HIGH);
    configuration &= ~(1<< CONFIG_OBSTACLE);
  }

  //  Network managment
  while(Serial.available())
  {
    int c = Serial.read();
    if(c >= 0) NETWORK.parseChar(c);
  }
}
//

//
int get_IR (uint16_t value)
{
  int distance = 2076.0 / (value - 11.0);
  if(distance > 30) return 31;
  else if(distance < 4) return -2;
  else return distance;
}
//

