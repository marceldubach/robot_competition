#include "Arduino.h"
#include "UltrasoundSensors.h"


UltrasoundSensors::UltrasoundSensors(byte pTrigger[], byte pEcho[], int nSensors){
   for(int i=0; i<nSensors; i++){
        pinTrigger[i] = pTrigger[i];
        pinEcho[i] = pEcho[i];

        pinMode(pinTrigger[i], OUTPUT);
        pinMode(pinEcho[i], INPUT);
   }
}

long UltrasoundSensors::readSensor(byte trigger, byte echo){
    //digitalWrite(trigger,LOW);
    //delayMicroseconds(5);
    //digitalWrite(trigger,HIGH);
    //delayMicroseconds(10);
    //digitalWrite(trigger,LOW);
    long duration = 1000; //pulseIn(echo, HIGH);
    //Serial.println(duration);
    return duration;
}

double UltrasoundSensors::convertToDistance(long duration){
    double distance = (double) ((duration/2)/29.1);
    return distance;
}

void UltrasoundSensors::readUS(byte pinTrigger[], byte pinEcho[], double results[], int r_size){
    for(int i=0; i<r_size; i++){
        long duration = readSensor(pinTrigger[i], pinEcho[i]);
        double distance = convertToDistance(duration);
        Serial.print("Distance");
        Serial.println(distance);
        results[i] = distance;
    }
}
