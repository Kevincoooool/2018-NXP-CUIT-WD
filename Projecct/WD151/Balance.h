#ifndef __BALANCE_H__
#define __BALANCE_H__
#include "common.h"
#include "headfile.h"

#define Zero_Angle 2.5f	// 蓝色电池
//#define Zero_Angle 22.0f	// 白色电池

//typedef struct
//{
//	float X;			 //比例系数
//	float Y;			 //积分系数
//	float Z;		 	 //微分系数

//}S_FLOAT_XYZ ; 

//typedef struct
//{
//	int16 X;			 //比例系数
//	int16 Y;			 //积分系数
//	int16 Z;		 	 //微分系数

//}S_INT16_XYZ ; 

//typedef struct
//{
//	int32 X;			 //比例系数
//	int32 Y;			 //积分系数
//	int32 Z;		 	 //微分系数

//}S_INT32_XYZ ; 

//extern S_FLOAT_XYZ 
//	GYRO_Real,		// 陀螺仪转化后的数据
//	ACC_Real,		// 加速度计转化后的数据
//	Attitude_Angle,	// 当前角度 
//	Last_Angle,		// 上次角度
//	Target_Angle;	// 目标角度
//	

//extern S_INT16_XYZ
//	GYRO,			// 陀螺仪原始数据
//	GYRO_Offset,	// 陀螺仪温飘
//	GYRO_Last,		// 陀螺仪上次数据
//	ACC, 			// 加速度计数据
//	ACC_Offset,		// 加速度计温飘
//	ACC_Last;		// 加速度计上次数据
//extern S_INT32_XYZ
//	Tar_Ang_Vel,	// 目标角速度
//	Tar_Ang_Vel_Last;	// 上次目标角速度

//extern int32 
//	Speed_Now,		// 当前实际速度
//	Speed_Min,		// 左右最小速度
//	Speed_Set, 		// 目标设定速度
//	Theory_Duty,
//	Direct_Last,
//	Speed_Diff,		// 当前实际速度
//	Vel_Set,		// 目标转向角速度
//	Direct_Parameter,
//	Radius;

extern uint8 Point;
extern int32 Difference;
float  Turn_Out_Filter(float turn_out);
extern char Speed_Flag, Angle_Flag, Ang_Velocity_Flag, DMP_Flag,MPU_Flag,JS_Flag;
//void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC);
void Balance_Control(void);


#endif
