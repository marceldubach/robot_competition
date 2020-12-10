#include "Motors.h"
#include "Arduino.h"

// Constuctor
Motors::Motors(bool MotorEnable = false)
{
    pinMode(avSpeed1, INPUT); //analog input -> average speed motor1
    pinMode(avSpeed2, INPUT); //analog input -> average speed motor1

    pinMode(pwm1, OUTPUT);    //PWM pin motor1
    pinMode(enable1, OUTPUT); //enable pin motor1

    pinMode(pwm2, OUTPUT);    //PWM pin motor2
    pinMode(enable2, OUTPUT); //enable pin motor2

    if (MotorEnable)
    {
        digitalWrite(enable1, HIGH); //enable motor1
        digitalWrite(enable2, HIGH); //enable motor2
    }
    else
    {
        digitalWrite(enable1, LOW);
        digitalWrite(enable2, LOW);
    }

    //TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
}

void Motors::commandMotors(int *avgSpeedMotorLeft, int *avgSpeedMotorRight, int commandMotorLeft, int commandMotorRight , bool MotorEnable = false)
{
    commandLeft = 0;
    commandRight = 0;

    avgSpeedLeft = 0;
    avgSpeedRight = 0;

    commandLeft = commandMotorLeft;
    commandRight = commandMotorRight;

    if (MotorEnable)
    {
        digitalWrite(enable1, HIGH);
        digitalWrite(enable2, HIGH);

        analogWrite(pwm1, commandLeft);
        analogWrite(pwm2, commandRight);

        avgSpeedLeft = analogRead(avSpeed1);
        avgSpeedRight = analogRead(avSpeed2);
    }
    else
    {
        digitalWrite(enable1, LOW);
        digitalWrite(enable2, LOW);

        analogWrite(pwm1, commandLeft);
        analogWrite(pwm2, commandRight);
    }

    avgSpeedMotorLeft = &avgSpeedLeft;          // give avgSpeed address to the pointer => point to same value
    avgSpeedMotorRight = &avgSpeedRight;
}