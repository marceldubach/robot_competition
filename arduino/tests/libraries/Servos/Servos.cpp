#include "Arduino.h"
#include "Servos.h" // custom library
#include "Servo.h"  // Arduino library

//constructor
Servos::Servos()
{
    // Create servo objects
    Servo myHighTorqueServo; // create servo object to control high torque servo
    Servo myMicroServoLeft;  // create servo object to control micro left
    Servo myMicroServoRight; // create servo object to control micro right
    Servo myCamServo;        // create servo object to control cam servo
    Servo myBackDoorServo;   // create servo object to control back door servo

    //attach pins to the servo objects
    myHighTorqueServo.attach(11, 400, 2550); // 400us-2550us DFROBOT high torque
    myMicroServoLeft.attach(10, 900, 2100);  // (pin, min, max) for HC-82 left
    myMicroServoRight.attach(9, 900, 2100);  // (pin, min, max) for HC-82 right
    myCamServo.attach(8, 750, 2250);         // (pin, min, max) for cam
    myBackDoorServo.attach(7, 750, 2250);    // (pin, min, max) for back door

    //define pins for Gripper Ultrasound Sensor
    byte pinTrigger = 37;
    byte pinEcho = 36;
    pinMode(pinTrigger, OUTPUT);
    pinMode(pinEcho, INPUT);
}

void Servos::grabBottle()
{
}

void Servos::emptyContainer()
{
}

void Servos::servosDrivingPos()
{
}

void Servos::closeGripper(int valueLeft = 40, int valueRight = 160)
{
    myMicroServoLeft.write(valueLeft);   // 40
    myMicroServoRight.write(valueRight); // 160
    delayMircoseconds(1000);
}

void Servos::openGripper(int valueLeft = 80, int valueRight = 120)
{
    myMicroServoLeft.write(valueLeft);   // 80
    myMicroServoRight.write(valueRight); // 120
    delayMircoseconds(1000);
}

void Servos::raiseGripper(int value = 160)
{
    myHighTorqueServo.write(value); // 160
    delayMircoseconds(1000);
}

void Servos::lowerGripper(int value = 15)
{
    myHighTorqueServo.write(value); // 15
    delayMircoseconds(1000);
}

bool Servos::isBottleGrabbed()
{
    digitalWrite(pinTrigger, LOW);
    delayMicroseconds(5);
    digitalWrite(pinTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger, LOW);

    unsigned long duration = pulseIn(pinEcho, HIGH);
    double distances = (duration / 2) / 29.1;

    if(disance >= )
    return bottleCatched;
}
