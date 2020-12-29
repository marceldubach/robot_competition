#ifndef ULTRASOUNDSENSORS_H
#define ULTRASOUNDSENSORS_H

// #include "Arduino.h" //askip faudrait pas mettre parce que ça polue le namespace

class UltrasoundSensors
{
public:
    // size of 7 sensors hardcoded
    UltrasoundSensors();
    void readUS(int *commandMotorLeft, int *commandMotorRight, int braitenberg[], int thresholdUS);
    double ReadOneUSReadOneUS(int currentUS, int thresholdUS);

private:
    byte pinTrigger[7];
    byte pinEcho[7];
};

#endif
