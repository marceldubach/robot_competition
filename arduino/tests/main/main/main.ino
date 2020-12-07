/*
 * This is the Arduino main code, here we implement the arduino state machine 
 * and call the appropriate function for each state.
 * 
 * Olivier
 */

#include <Arduino.h>; 
#include "UltrasoundSensors.h";


uint8_t pTrigger[7] =  {25, 23, 27, 29, 31, 33, 35};
uint8_t pEcho[7] =     {24, 22, 26, 28, 30, 32, 34};
UltrasoundSensors(pTrigger, pEcho, 7);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
