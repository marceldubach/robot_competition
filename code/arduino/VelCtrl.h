#ifndef VelCtrl_h
#define VelCtrl_h
#define PI 3.1415
#include "Arduino.h"

void set_Commands(bool enableMotors, int ctrlRight, int ctrlLeft,byte pwmRight, byte pwmLeft, byte enableRight, byte enableLeft);

void calculate_Commands(int& ctrlLeft,int& ctrlRight, double x, double y, double theta, double ref_x, double ref_y, double max_dist[]);

void calculate_Commands(int& ctrlLeft,int& ctrlRight, double x, double y, double theta, double ref_x, double ref_y);

#endif
