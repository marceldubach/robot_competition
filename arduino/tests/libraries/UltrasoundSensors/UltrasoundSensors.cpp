#include "Arduino.h"
#include "UltrasoundSensors.h"

UltrasoundSensors::UltrasoundSensors(byte pTrigger[], byte pEcho[])
{
    for (int i = 0; i < 7; i++)
    {
        pinTrigger[i] = pTrigger[i];
        pinEcho[i] = pEcho[i];

        pinMode(pinTrigger[i], OUTPUT);
        pinMode(pinEcho[i], INPUT);
    }
}
void UltrasoundSensors::readUS(int commands[2], int braitenberg[14], long interval_ms = 20, int thresholdUS = 100)
{
    unsigned long currentMillis = millis(); // retrieve current value of millis
    if (currentMillis - previousMillis >= interval_ms)
    {

        cli(); //stop interrupts

        previousMillis = currentMillis;

        for (int i = 0; i < 7; i++)
        { // Loop through the 7 US to read the values
            digitalWrite(trigger[i], LOW);
            delayMicroseconds(5);
            digitalWrite(trigger[i], HIGH);
            delayMicroseconds(10);
            digitalWrite(trigger[i], LOW);

            unsigned long duration = pulseIn(echo[i], HIGH);
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
                //Serial.print("CMD left: ");
                //Serial.println(commandLeft);
            }
            else if (j == 1)
            {
                commandRight = sumDotProd;
                //Serial.print("CMD right: ");
                //Serial.println(commandRight);
            }
        }
        commandLeft = 128 + commandLeft;
        commandRight = 128 - commandRight;

        if (commandLeft > 240)
        {
            commands[0] = 240;
        }
        if (commandLeft <= 10)
        {
            commands[0] = 10;
        }

        if (commandRight > 240)
        {
            commands[1] = 240;
        }
        if (commandRight < 10)
        {
            commands[1] = 10;
        }

        return;
    }
}

// OLD BRAITENBERG CODE FROM MARCEL BELOW

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
