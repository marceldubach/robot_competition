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

void Motors::commandMotors(int avgSpeedMotors[2], int commands[2], bool MotorEnable = false)
{
    commandLeft = 0;
    commandRight = 0;

    avgSpeedMotorLeft = 0;
    avgSpeedMotorRight = 0;

    commandLeft = commands[0];
    commandRight = commands[1];

    if (MotorEnable)
    {
        digitalWrite(enable1, HIGH);
        digitalWrite(enable2, HIGH);

        analogWrite(pwm1, commandLeft);
        analogWrite(pwm2, commandRight);

        avgSpeedMotorLeft = analogRead(avSpeed1);
        avgSpeedMotorRight = analogRead(avSpeed2);
    }
    else
    {
        digitalWrite(enable1, LOW);
        digitalWrite(enable2, LOW);

        analogWrite(pwm1, commandLeft);
        analogWrite(pwm2, commandRight);
    }

    avgSpeedMotors[0] = avgSpeedMotorLeft;
    avgSpeedMotors[1] = avgSpeedMotorRight;
}