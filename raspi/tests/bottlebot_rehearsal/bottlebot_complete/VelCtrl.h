#ifndef VelCtrl_h
#define VelCtrl_h

#include "Arduino.h"


#define PI 3.1415

void set_Commands(bool enableMotors, int ctrlRight, int ctrlLeft,byte pwmRight, byte pwmLeft, byte enableRight, byte enableLeft);

void calculate_Commands(int& ctrlLeft,int& ctrlRight, double x, double y, double theta, double ref_x, double ref_y);

#endif
