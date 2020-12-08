#ifndef ULTRASOUNDSENSORS_H
#define ULTRASOUNDSENSORS_H

// #include "Arduino.h" //askip faudrait pas mettre parce que ça polue le namespace

class UltrasoundSensors
{
public:
    // size of 7 sensors hardcoded
    UltrasoundSensors(byte pTrigger[7], byte pEcho[7]);
    void readUS(int commands[2], int braitenberg[14], unsigned long interval_ms = 20, int thresholdUS = 100);

private:
    byte pinTrigger[7];
    byte pinEcho[7];

    // OLD BRAITENBERG CODE FROM MARCEL BELOW

    // long readSensor(byte trigger, byte echo);
    // double convertToDistance(long sensorReading);
};

#endif