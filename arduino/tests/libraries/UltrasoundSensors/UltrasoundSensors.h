#ifndef UltrasoundSenors_h
#define UltrasoundSenors_h

#include "Arduino.h"

class UltrasoundSensors{
    public:
        UltrasoundSensors(byte pTrigger[7], byte pEcho[7], int nSensors);
        void readUS(byte pinTrigger[], byte pinEcho[], double results[], int r_size);

    private:
        byte pinTrigger[7];
        byte pinEcho[7];

        long readSensor(byte trigger, byte echo);
        double convertToDistance(long sensorReading);
};

#endif
