#ifndef __PID_H__
#define __PID_H__
#include "common.h"
#include "headfile.h"
#include "filter.h"
#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5

typedef struct PID
{
	float SumError;	//误差累计	
	int32 LastError;	//Error[-1]
	int32 PrevError;	//Error[-2]	
	int32 LastData;	//Speed[-1]
	int32 Dis_Err;//微分量
	float Dis_Error_History[5];//历史微分量
	Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲
} PID;

extern PID Speed_PID, Angle_PID, Ang_gyro_PID, Turn_gyro_PID, Turn_PID, Distance_PID;	//电机的PID参数结构体
extern float Speed[4], Angle[4], Ang_gyro[5], Turn_gyro[5], Turn[5][4];

// PID参数初始化
void PID_Parameter_Init(PID *sptr);


// 位置式动态PID控制
int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint);

// 位置式PID控制
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);

// 增量式PID控制
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point);

#endif
