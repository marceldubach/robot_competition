#ifndef MOTORS_H
#define MOTORS_H

//Motor1 ports
#define enable1 51
#define pwm1 3
#define avSpeed1 A1

//Motor2 ports
#define enable2 50
#define pwm2 2
#define avSpeed2 A3

class Motors
{
private:
    int commandLeft, commandRight;
    int avgSpeedLeft, avgSpeedRight;

public:
    Motors(bool MotorEnable = false);
    void commandMotors(int &avgSpeedMotorLeft, int &avgSpeedMotorRight, int commandMotorLeft, int commandMotorRight , bool MotorEnable = false);
};

#endif