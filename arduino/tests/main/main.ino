/*
   This is the Arduino main code, here we implement the arduino state machine
   and call the appropriate function for each state.

   Lorenzo/Marcel/Olvier - Robot Contest 2020
*/

#include "Arduino.h"
#include "UltrasoundSensors.h"
#include "Motors.h"
#include "Servos.h"

int currentState = 1;                 // define current state of state machine
int thresholdUS = 100;                // define the US max distance ( default : 100 cm)
const unsigned long interval_ms = 60; // define interval between US sampling (lowest possible)
unsigned long currentMillis, previousMillis;
int braitenberg[14] = {700, 100, 500, -500, -500, 500, 10,  // LEFT      // define variables for braitenberg
                       600, 250, 50, -500, -500, 600, 120}; // RIGHT;
int *commandMotorLeft;                                      // define pointer for motor input LEFT and RIGHT
int *commandMotorRight;
int *avgSpeedMotorLeft; // define pointer to store the motor average speed reading from Hall sensors
int *avgSpeedMotorRight;
bool MotorEnable; // toggles the Enable pins of both motors
enum states       // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2
};
unsigned long startTime;

void setup() // put your setup code here, to run once:
{
  Serial.begin(38400);
  MotorEnable = false;
  currentState = moving;
}

// CONSTRUCTORS
UltrasoundSensors MyUltrasoundSensors;
Motors MyMotors;
Servos MyServos;

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

    MyMotors.commandMotors(avgSpeedMotorLeft, avgSpeedMotorRight, *commandMotorLeft, *commandMotorRight, MotorEnable); // derefence command pointer for value
    /*
    Serial.print("Left : ");
    Serial.print(commandMotorLeft);
    Serial.print("   Right :");
    Serial.println(commandMotorRight);
    */
    break;

  case grabing:
    MyServos.approachBottle(); // lowers and opens claws for approach
    MyServos.grabBottle();     // tries to catch the bottle max 3 times

    Serial.print("number of bottles in container ");
    Serial.println(MyServos.bottleCount);

    if (MyServos.successfulCatch)
    {
      currentState = idling; // change in final code
    }
    else
    {
      currentState = idling; // we'll need to do smth else
    }

    break;

  default:
    // error here
    break;
  }
}