#include "Arduino.h"
#include "Servos.h" // custom library (calls Servo.h too)

//constructor
Servos::Servos()
{
    /*
    // create subclasses
    myHighTorqueServo;  // create servo object to control high torque servo
    myMicroServoLeft;   // create servo object to control micro left
    myMicroServoRight;  // create servo object to control micro right
    myCamServo;         // create servo object to control cam servo
    myBackDoorServo;    // create servo object to control back door servo
*/
    // attach pins to the servo objects
    myHighTorqueServo.attach(11, 400, 2550); // 400us-2550us DFROBOT high torque
    myMicroServoLeft.attach(10, 900, 2100);  // (pin, min, max) for HC-82 left
    myMicroServoRight.attach(9, 900, 2100);  // (pin, min, max) for HC-82 right
    myCamServo.attach(8, 750, 2250);         // (pin, min, max) for cam
    myBackDoorServo.attach(7, 750, 2250);    // (pin, min, max) for back door

    // define pins for Gripper Ultrasound Sensor
    byte pinTrigger = 37;
    byte pinEcho = 36;
    pinMode(pinTrigger, OUTPUT);
    pinMode(pinEcho, INPUT);

    // initialise variables
    bottleCount = 0;
    bottleCatchTries = 0;
}

void Servos::approachBottle()
{
    lowerGripper();
    openGripper();
}

void Servos::grabBottle()
{
    closeGripper();
    isBottleGrabbed();
    successfulCatch = false; //reset

    if (bottleCatched = true)
    {
        raiseGripper();
        openGripper(60, 140);   // open but not fully
        bottleCount++;          // increase catched bottle count
        successfulCatch = true; // sucessful catch
        servosDrivingPos();
    }
    else if (bottleCatched = false && bottleCatchTries < 3) // no more than 3 tries
    {
        openGripper();
        bottleCatchTries++; // increment tentative counter
        grabBottle();       // restart procedure
    }
    else
    {
        bottleCatchTries = 0;    // reset counter (mission failed, we'll get them next time)
        servosDrivingPos();      // get in driving mode
        successfulCatch = false; // unsucessful catch
    }
}

void Servos::emptyContainer(int backDoorOpen = 60, int backDoorClosed = 165, unsigned long interval_ms = 3000)
{
    unsigned long currentMillis, startMillis; // to compute time interval

    myBackDoorServo.write(backDoorOpen);
    delayMicroseconds(oneSecond);

    startMillis = millis();
    while (currentMillis - startMillis >= interval_ms)
    {
        currentMillis = millis();
        myCamServo.write(0);
        delayMicroseconds(halfSecond);
        myCamServo.write(70);
        delayMicroseconds(halfSecond);
    }

    myBackDoorServo.write(backDoorClosed);
    delayMicroseconds(oneSecond);

    bottleCount = 0; //reset captured bottle count
}

void Servos::servosDrivingPos() //raise gripper and open claw
{
    raiseGripper(100); // needs to be tuned
    openGripper();     // needs to be tuned
}

void Servos::closeGripper(int valueLeft = 40, int valueRight = 160)
{
    myMicroServoLeft.write(valueLeft);   // 40
    myMicroServoRight.write(valueRight); // 160
    delayMicroseconds(oneSecond);
}

void Servos::openGripper(int valueLeft = 80, int valueRight = 120)
{
    myMicroServoLeft.write(valueLeft);   // 80
    myMicroServoRight.write(valueRight); // 120
    delayMicroseconds(oneSecond);
}

void Servos::raiseGripper(int value = 160)
{
    myHighTorqueServo.write(value); // 160
    delayMicroseconds(oneSecond);
}

void Servos::lowerGripper(int value = 15)
{
    myHighTorqueServo.write(value); // 15
    delayMicroseconds(oneSecond);
}

bool Servos::isBottleGrabbed()
{
    //cli(); //stop interrupts

    digitalWrite(pinTrigger, LOW);
    delayMicroseconds(5);
    digitalWrite(pinTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger, LOW);

    unsigned long duration = pulseIn(pinEcho, HIGH,4000);
    double distance = (duration / 2) / 29.1;

    //sei(); // restart interrupts

    if (distance <= 30) // distance threshold to determine if bottle is grabbed
    {
        bottleCatched = true;
    }
    else
    {
        bottleCatched = false;
    }

    return bottleCatched;
}