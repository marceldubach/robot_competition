/*
   This is the Arduino main code, here we implement the arduino state machine
   and call the appropriate function for each state.

   Olivier - Robot Contest 2020
*/

#include <Arduino.h>
#include "UltrasoundSensors.h"
#include "Motors.h"
//#include <stdio.h>

int currentState = 1;                                                   // define current state of state machine
int thresholdUS = 100;                                                  // define the US max distance ( default : 100 cm)
byte pTrigger[7] = {25, 23, 27, 29, 31, 33, 35};
byte pEcho[7] = {24, 22, 26, 28, 30, 32, 34};                           // define TRIGGER & ECHO pins for US
const unsigned long interval_ms = 20;                                   // define interval between US sampling (default : 20ms)
int braitenberg[14] = {700, 100, 500, -500, -500, 500, 10, // LEFT      // define variables for braitenberg
                       600, 250, 50, -500, -500, 600, 120}; // RIGHT;                      
int commands[2] = {0, 0};                                               // define motor input LEFT then RIGHT
int avgSpeedMotors[2];                                       // store the motor average speed reading from Hall sensors          
bool MotorEnable;                                                       // toggles the Enable pins of both motors
enum states                                                             // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2
};

void setup()  // put your setup code here, to run once:
{ 
  Serial.begin(9600);
  MotorEnable = true;
}

// CONSTRUCTORS
UltrasoundSensors MyUltrasoundSensors(pTrigger[7], pEcho[7]);
Motors MyMotors;

void loop()
{
  switch (currentState)
  {
  case idling:
    // idling here   
    break;
        
  case moving:
    MyUltrasoundSensors.readUS(commands[2], braitenberg[14], interval_ms, thresholdUS);
    MyMotors.commandMotors(avgSpeedMotors[2], commands[2], MotorEnable);
    break;

  case grabing:
    // grabbing sequence
    break;

  default:
    // error here
    break;
  }
}
