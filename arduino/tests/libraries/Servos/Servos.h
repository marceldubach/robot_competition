#ifndef SERVOS_H
#define SERVOS_H

// define pins here



class Servos
{
private: 
    //methods
    void Servos::closeGripper(int valueLeft, int valueRight);   // close gripper to given angle
    void Servos::openGripper(int valueLeft, int valueRight);    // open gripper to given angle
    void Servos::raiseGripper(int value);                       // raise gripper to given angle
    void Servos::lowerGripper(int value);                       // lower gripper to given angle
    bool Servos::isBottleGrabbed();                             // check US if there is a bottle in the gripper

    //sub classes
    myHighTorqueServo;  // create servo object to control high torque servo
    myMicroServoLeft;   // create servo object to control micro left
    myMicroServoRight;  // create servo object to control micro right
    myCamServo;         // create servo object to control cam servo
    myBackDoorServo;    // create servo object to control back door servo

    // variables
    byte pinTrigger;
    byte pinEcho;
    bool bottleCatched;

public:
    Servos(/* args */);                         // constructor
    void Servos::grabBottle();                  // launch grabbing sequence
    void Servos::emptyContainer();              // launch the emptying sequence (back door + cam)
    void Servos::servosDrivingPos();            // set position of Gripper to driving mode

};

#endif