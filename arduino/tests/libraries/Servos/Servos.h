#ifndef SERVOS_H
#define SERVO_H

// define pins here



class Servos
{
private: 
    void Servos::closeGripper();                // close gripper to given angle
    void Servos::openGripper();                 // open gripper to given angle
    void Servos::raiseGripper();                // raise gripper to given angle
    void Servos::lowerGripper();                // lower gripper to given angle
    bool Servos::isBottleGrabbed();             // check US if there is a bottle in the gripper

public:
    Servos(/* args */);                         // constructor
    void Servos::grabBottle();                  // launch grabbing sequence
    void Servos::emptyContainer();              // launch the emptying sequence (back door + cam)
    void Servos::servosDrivingPos();            // set position of Gripper to driving mode

};

#endif