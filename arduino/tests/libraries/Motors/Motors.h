#ifndef MOTORS_H
#define MOTORS_H

//MotorRight
#define enableRight 51
#define pwmRight 3
#define avSpeedRight A1

//MotorLeft
#define enableLeft 50
#define pwmLeft 2
#define avSpeedLeft A3

class Motors
{
private:
    int commandLeft, commandRight;
    int avgSpeedLeft, avgSpeedRight;

public:
    Motors();
    void commandMotors(int *avgSpeedMotorLeft, int *avgSpeedMotorRight, int commandMotorLeft, int commandMotorRight, bool MotorEnable);
    void getAvgSpeed(int *avgSpeedMotorLeft, int *avgSpeedMotorRight);
};

#endif