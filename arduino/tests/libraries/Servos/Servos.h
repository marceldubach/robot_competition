#ifndef SERVOS_H
#define SERVOS_H

#include "Servo.h" //needed for sub-class

#define oneSecond 1000000 //define second for better readability of delayMircoseconds() 0
#define halfSecond 500000

class Servos
{
private:
    // methods
    void closeGripper(int valueLeft = 40, int valueRight = 160); // close gripper to given angle
    void openGripper(int valueLeft = 80, int valueRight = 120);  // open gripper to given angle
    void raiseGripper(int value = 160);                          // raise gripper to given angle
    void lowerGripper(int value = 15);                           // lower gripper to given angle
    bool isBottleGrabbed();                                      // check US if there is a bottle in the gripper

    // create sub classes
    Servo myHighTorqueServo; // create servo object to control high torque servo
    Servo myMicroServoLeft;  // create servo object to control micro left
    Servo myMicroServoRight; // create servo object to control micro right
    Servo myCamServo;        // create servo object to control cam servo
    Servo myBackDoorServo;   // create servo object to control back door servo

    // variables
    byte pinTrigger;
    byte pinEcho;
    bool bottleCatched;

public:
    // constructor
    Servos();

    // methods
    void approachBottle();                                                                                  // approach the bottle with gripper open and down
    void grabBottle();                                                                                      // launch grabbing sequence
    void emptyContainer(int backDoorOpen = 60, int backDoorClosed = 165, unsigned long interval_ms = 3000); // launch the emptying sequence (back door + cam)
    void servosDrivingPos();                                                                                // set position of Gripper to driving mode

    // variables
    int bottleCount;      // count current bottles in containers
    int bottleCatchTries; // count the number of catching tentatives
    bool successfulCatch; // if the catch was successful or not
};

#endif