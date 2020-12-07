#ifndef UltrasoundSenors_h
#define UltrasoundSenors_h

#include "Arduino.h"

class UltrasoundSensors{
    public:
        // size of 7 sensors hardcoded
        UltrasoundSensors(byte pTrigger[7], byte pEcho[7]);
        void readUS(double results[7]);

    private:
        byte pinTrigger[7];
        byte pinEcho[7];
        unsigned long duration;
        // long readSensor(byte trigger, byte echo);
        // double convertToDistance(long sensorReading);
};

#endif