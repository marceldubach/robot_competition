#include "Motors.h"
#include "Arduino.h"

// Constuctor
Motors::Motors()
{
    pinMode(avSpeedRight, INPUT); //analog input -> average speed motor1
    pinMode(avSpeedLeft, INPUT);  //analog input -> average speed motor1

    pinMode(pwmRight, OUTPUT);    //PWM pin motor1
    pinMode(enableRight, OUTPUT); //enable pin motor1

    pinMode(pwmLeft, OUTPUT);    //PWM pin motor2
    pinMode(enableLeft, OUTPUT); //enable pin motor2

    TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
}

void Motors::commandMotors(int *avgSpeedMotorLeft, int *avgSpeedMotorRight, int commandMotorLeft, int commandMotorRight, bool MotorEnable)
{
    commandLeft = commandMotorLeft;
    commandRight = commandMotorRight;

    if (MotorEnable)
    {
        digitalWrite(enableRight, HIGH);
        digitalWrite(enableLeft, HIGH);

        analogWrite(pwmLeft, commandLeft);
        analogWrite(pwmRight, commandRight);
    }
    else if (MotorEnable == false)
    {
        digitalWrite(enableRight, LOW);
        digitalWrite(enableLeft, LOW);
    }

    avgSpeedLeft = analogRead(avSpeedLeft);
    avgSpeedRight = analogRead(avSpeedRight);

    *avgSpeedMotorLeft = avgSpeedLeft;
    *avgSpeedMotorRight = avgSpeedRight;
}