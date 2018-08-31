#ifndef __MYBALANCE_H__
#define __MYBALANCE_H__
#include "common.h"
#include "headfile.h"

int Balance_1(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
void My_Control(void);
extern float ACC_Angle,Car_Angle,Angle_Speed;
extern int Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z;
void My_PIDInit(void);
#endif
