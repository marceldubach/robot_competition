#include "Arduino.h"
#include "UltrasoundSensors.h"

// Constructor
UltrasoundSensors::UltrasoundSensors()
{
    byte pTrigger[7] = {25, 23, 27, 29, 31, 33, 35}; // define TRIGGER pins
    byte pEcho[7] = {24, 22, 26, 28, 30, 32, 34};    // define ECHO pins

    for (int i = 0; i < 7; i++)
    {
        pinTrigger[i] = pTrigger[i];
        pinEcho[i] = pEcho[i];

        pinMode(pinTrigger[i], OUTPUT);
        pinMode(pinEcho[i], INPUT);
    }
}

// Method
void UltrasoundSensors::readUS(int *commandMotorLeft, int *commandMotorRight, int braitenberg[], int thresholdUS = 100)
{
    double distances[7];
    int thresholdArray[7];
    int commandLeft, commandRight, sumDotProd;

    cli(); //stop interrupts

    for (int i = 0; i < 7; i++)
    { // Loop through the 7 US to read the values
        digitalWrite(pinTrigger[i], LOW);
        delayMicroseconds(5);
        digitalWrite(pinTrigger[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(pinTrigger[i], LOW);

        unsigned long duration = pulseIn(pinEcho[i], HIGH);
        distances[i] = (duration / 2) / 29.1;
    }

    sei(); // restart interrupts

    for (int i = 0; i < 7; i++)
    {
        if (distances[i] < thresholdUS)
        {
            thresholdArray[i] = 1;
        }
        else
        {
            thresholdArray[i] = 0;
        }
    }
    commandLeft = 0;
    commandRight = 0;

    // multiplie sensor input by braitenberg array and get outputs using a threshold on the sensor values
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

    if (commandLeft > 240)
    {
        commandLeft = 240;
    }
    if (commandLeft <= 10)
    {
        commandLeft = 10;
    }

    if (commandRight > 240)
    {
        commandRight = 240;
    }
    if (commandRight < 10)
    {
        commandRight = 10;
    }

    *commandMotorLeft = commandLeft;
    *commandMotorRight = commandRight;
}