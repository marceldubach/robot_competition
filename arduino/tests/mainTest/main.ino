/*
   This is the Arduino main code, here we implement the arduino state machine
   and call the appropriate function for each state.

   Lorenzo/Marcel/Olvier - Robot Contest 2020
*/

#include <Arduino.h>
#include <Servo.h>
#include "UltrasoundSensors.h"
#include "Motors.h"
#include "Servos.h"

int currentState;                // define current state of state machine
int thresholdUS;                 // define the US max distance
const unsigned long interval_ms; // define interval between US sampling (lowest possible)
unsigned long currentMillis, previousMillis;
int braitenberg[14] = {1400, 200, 1000, -1000, -1000, 100, 500,  // LEFT      // define variables for braitenberg
                       1200, 500, 100, -1000, -1000, 1200, 240}; // RIGHT;
int commandMotorLeft;                                            // define variable for motor input LEFT and RIGHT
int commandMotorRight;
int avgSpeedMotorLeft; // define variable to store the motor average speed reading from Hall sensors
int avgSpeedMotorRight;
bool MotorEnable; // toggles the Enable pins of both motors
enum states       // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2,
  improvedMoving = 3
};
unsigned long startTime;

double distance[7];
int currentUS;

// CONSTRUCTORS
UltrasoundSensors myUltrasoundSensors;
Motors myMotors;
//Servos myServos; // problem here

void setup() // put your setup code here, to run once:
{
  Serial.begin(38400);
  MotorEnable = false;
  currentState = improvedMoving;
  currentUS = 0;
  thresholdUS = 50;
  interval_ms = 60;

  for (i = 0; i++; i < 7)
  {
    distance[i] = 0; //initialize empty arrays
  }
}

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
      myUltrasoundSensors.readUS(&commandMotorLeft, &commandMotorRight, braitenberg, thresholdUS);
      myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);
    }

    // does not work if called too fast
    //myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);

    currentState = moving;
    break;

  case grabing:
    //myServos.approachBottle();

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

  case improvedMoving: // sample US one by one every 10 ms

    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) // 7*10ms >= than the recommended 60ms
    {
      previousMillis = currentMillis;

      // update US value
      distance[currentUS] = myUltrasoundSensors.readOneUS(currentUS, thresholdUS);
      
      // update Braitenberg control values
      for (int j = 0; j < 2; j++)
      {
        sumDotProd = 0; //reset for next row x col product
        for (int i = 0; i < 7; i++)
        {
          sumDotProd += thresholdArray[i] / distances[i] * braitenberg[j * 7 + i]; // (j * 7 + i) gives array position
        }
        if (j == 0)
        {
          commandLeft = sumDotProd;
        }
        else if (j == 1)
        {
          commandRight = sumDotProd;
        }
      }
      commandLeft = 128 + commandLeft;
      commandRight = 128 - commandRight;

      if (commandLeft > 240) commandLeft = 240;
      if (commandLeft <= 10) commandLeft = 10;
      if (commandRight > 240) commandRight = 240;
      if (commandRight < 10) commandRight = 10;

      // apply new value to motors
      myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);

      // increment currentUS
      currentUS = (currentUS + 1) %7;
    }
    else // still get motor speed
    {
      myMotors.getAvgSpeed(&avgSpeedMotorLeft, &avgSpeedMotorRight);
    }

    currentState = improvedMoving;
    break;

  default:
    // error here
    break;
  }
}
