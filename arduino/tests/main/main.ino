/*
   This is the Arduino main code, here we implement the arduino state machine
   and call the appropriate function for each state.

   Olivier - Robot Contest 2020
*/

#include <Arduino.h>;
#include "UltrasoundSensors.h";

int currentState;       // define current state of state machine
int thresholdUS;        // define the US max distance ( default : 100 cm)
uint8_t pTrigger[7];    // define TRIGGER pins for US
uint8_t pEcho[7];       // define ECHO pins for US
const long interval_ms; // define interval between US sampling (default : 20ms)
int braitenberg[14];    // define variables for braitenberg
int commands[2];        // define motor input LEFT then RIGHT
enum states             // define states for state machine
{ 
  idling = 0 moving = 1,
  grabing = 2
};

UltrasoundSensors(pTrigger, pEcho, 7);

void setup()
{ // put your setup code here, to run once:
  Serial.begin(9600);
  uint8_t pTrigger[7] = {25, 23, 27, 29, 31, 33, 35};
  uint8_t pEcho[7] = {24, 22, 26, 28, 30, 32, 34};
  currentState = 1 commands = {0, 0};
  braitenberg = {700, 100, 500, -500, -500, 500, 10 // LEFT
                 600,
                 250, 50, -500, -500, 600, 120}; // RIGHT
}

void loop()
{
  switch (currentState)
  {
  case idling:

  case moving:
    readUS(commands, braitenberg, interval_ms)
        analogWrite(pwm2, commands[0]); // dutyCicle in (0,255)  //a digital signal(square wave) as output
    analogWrite(pwm1, commands[1]);
    break;
  }
case grabing:
  // grabbing sequence
  break;

case default:
  // error here
  break;
}
}
