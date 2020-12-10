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
const unsigned long interval_ms = 60;                                   // define interval between US sampling (lowest possible)
unsigned long currentMillis, previousMillis;
int braitenberg[14] = {700, 100, 500, -500, -500, 500, 10, // LEFT      // define variables for braitenberg
                       600, 250, 50, -500, -500, 600, 120}; // RIGHT;                      
int commandMotorLeft;                                                   // define motor input LEFT then RIGHT
int commandMotorRight;
int avgSpeedMotorLeft;                                                  // store the motor average speed reading from Hall sensors
int avgSpeedMotorRight;          
bool MotorEnable;                                                       // toggles the Enable pins of both motors
enum states                                                             // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2
};
unsigned long startTime;

void setup()  // put your setup code here, to run once:
{ 
  Serial.begin(38400);
  MotorEnable = false;
  currentState = moving;
  startTime = millis();
}

// CONSTRUCTORS
UltrasoundSensors MyUltrasoundSensors(pTrigger, pEcho);
Motors MyMotors;

void loop()
{
  switch (currentState)
  {
  case idling:
    // idling here   
    break;
        
  case moving:
    currentMillis = millis(); // retrieve current value of millis
    if (currentMillis - previousMillis >= interval_ms)
    {
      previousMillis = currentMillis;
      MyUltrasoundSensors.readUS(commandMotorLeft, commandMotorRight, braitenberg, thresholdUS);
    }
    
    MyMotors.commandMotors(avgSpeedMotorLeft, avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);
    /*
    Serial.print("Left : ");
    Serial.print(commandMotorLeft);
    Serial.print("   Right :");
    Serial.println(commandMotorRight);
    */
    break;

  case grabing:
    // grabbing sequence
    break;

  default:
    // error here
    break;
  }
}
