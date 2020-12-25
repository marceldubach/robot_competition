/*
   This is the Arduino main code, here we implement the arduino state machine
   and call the appropriate function for each state.

   Lorenzo/Marcel/Olvier - Robot Contest 2020
*/

#include "Arduino.h"
#include "UltrasoundSensors.h"
#include "Motors.h"
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include "Servos.h"
#define dutyCycle 230 // 15 to 240

byte duty = dutyCycle; 
int count = 0;
unsigned long startTime;
unsigned long dT;
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float gSensScaleFactor = 131.00; //[LSB/(°/s)] gain at ±250 configuration DEFAULT ONE
float gNoiseMean = 0.86; // mean noise on gyroscope gz lecture, averaged over 1000 data points 
float scaledGz;
float aSensScaleFactor = 16384; // [LSB/g] gain at ±2 configuration DEFAULT ONE
float accNoiseMean = -0.04; // mean noise on accelerometre ax lecture, averaged over 1000 data points 
float scaledAx;

int currentState = 1;                 // define current state of state machine
int thresholdUS = 30;                // define the US max distance ( default : 30 cm)
const unsigned long interval_ms = 60; // define interval between US sampling (lowest possible)
unsigned long currentMillis, previousMillis;
int braitenberg[14] = {1400, 200, 1000, -1000, -1000, 100, 500,  // LEFT      // define variables for braitenberg
                       1200, 500, 100, -1000, -1000, 1200, 240
                      }; // RIGHT;
int commandMotorLeft;  // define pointer for motor input LEFT and RIGHT
int commandMotorRight;
int avgSpeedMotorLeft; // define variable to store the motor average speed reading from Hall sensors
int avgSpeedMotorRight;
bool MotorEnable; // toggles the Enable pins of both motors
enum states       // define states for state machine
{
  idling = 0,
  moving = 1,
  grabing = 2
};

// CONSTRUCTORS
//UltrasoundSensors myUltrasoundSensors;
Motors myMotors;


void setup() // put your setup code here, to run once:
{
  Wire.begin();
  Serial.begin(38400);
  while (!Serial) continue;
  accelgyro.initialize();
  while(!accelgyro.testConnection()) continue;
  startTime = millis();
  MotorEnable = false;
  currentState = moving;
}

void loop()
{

  // if the pi communicates to stop
   if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "stop"){
      digitalWrite(enableRight, LOW); //enable motor1                  
      digitalWrite(enableLeft, LOW); //enable motor2
    }
   }
   
  if (millis()>startTime + 18){ // 20ms as DT
    dT = millis() - startTime;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    scaledGz = gz/gSensScaleFactor - gNoiseMean;
    scaledAx = ax/aSensScaleFactor - accNoiseMean;
    //Serial.println(millis()-startTime);
    startTime = millis();
    StaticJsonDocument<120> doc;  // to verify
    doc["gyro"] = scaledGz;
    doc["accel"] = scaledAx;
    doc["dT"] = dT;
    myMotors.getAvgSpeed(&avgSpeedMotorLeft, &avgSpeedMotorRight);
    JsonArray motorSpeed = doc.createNestedArray("motorSpeed");
    motorSpeed.add(avgSpeedMotorRight);
    motorSpeed.add(avgSpeedMotorLeft);
    // Send the JSON document over the serial port
    serializeJson(doc, Serial);
    Serial.println();
  }
  
  switch (currentState)
  {
    case idling:
      // idling here
      break;

    case moving:
    /*
      currentMillis = millis(); // retrieve current value of millis
      if (currentMillis - previousMillis >= interval_ms)
      { 
        
        previousMillis = currentMillis;
        //myUltrasoundSensors.readUS(&commandMotorLeft, &commandMotorRight, braitenberg, thresholdUS);
        //myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);
      }

      // does not work if called too fast
      //myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, commandMotorLeft, commandMotorRight, MotorEnable);

      currentState = moving;
      */
      // if not working put inside if()
      myMotors.commandMotors(&avgSpeedMotorLeft, &avgSpeedMotorRight, duty, 255 - duty, MotorEnable);

      break;
      
    /*
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
    */
    default:
      // error here
      break;
  }
  
}
