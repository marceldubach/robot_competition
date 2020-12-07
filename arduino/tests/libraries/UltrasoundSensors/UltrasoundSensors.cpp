#include "Arduino.h"
#include "UltrasoundSensors.h"


UltrasoundSensors::UltrasoundSensors(byte pTrigger[], byte pEcho[]){
   for(int i=0; i < 7; i++){
        pinTrigger[i] = pTrigger[i];
        pinEcho[i] = pEcho[i];

        pinMode(pinTrigger[i], OUTPUT);
        pinMode(pinEcho[i], INPUT);
   }
}

void UltrasoundSensors::readUS(double results[7]){
    for(int i=0; i < 7; i++){
      digitalWrite(trigger[i], LOW);
      delayMicroseconds(5);
      digitalWrite(trigger[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger[i], LOW);

      unsigned long duration = pulseIn(echo[i], HIGH);
      results[i] = (duration / 2) / 29.1;
    }
}

/* long UltrasoundSensors::readSensor(byte trigger, byte echo){
    
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
} */
