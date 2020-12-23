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

int currentState = 1;                 // define current state of state machine
int thresholdUS = 100;                // define the US max distance ( default : 100 cm)
const unsigned long interval_ms = 60; // define interval between US sampling (lowest possible)
unsigned long currentMillis, previousMillis;
int braitenberg[14] = {1400, 200, 1000, -1000, -1000, 100, 500,  // LEFT      // define variables for braitenberg
                       1200, 500, 100, -1000, -1000, 1200, 240}; // RIGHT;
int commandMotorLeft;   // define variable for motor input LEFT and RIGHT
int commandMotorRight;
int avgSpeedMotorLeft;  // define variable to store the motor average speed reading from Hall sensors
int avgSpeedMotorRight;
bool MotorEnable;       // toggles the Enable pins of both motors
enum states             // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2
};
unsigned long startTime;

// CONSTRUCTORS
UltrasoundSensors myUltrasoundSensors;
Motors myMotors;
Servos myServos; // problem here


void setup() // put your setup code here, to run once:
{
  Serial.begin(38400);
  MotorEnable = false;
  currentState = grabing;
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
      /*
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
        */
        break;
    
    default:
      // error here
      break;
  }
}
