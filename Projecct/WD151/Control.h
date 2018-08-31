#ifndef _Control_h
#define _Control_h

#include "headfile.h"
#include "common.h"
#include "filter.h"
//定义电机端口
#define MOTOR_FTM   FTM2
#define MOTOR1_PWM  FTM2_CH0_PIN	// PTC1
#define MOTOR2_PWM  FTM2_CH1_PIN	// PTC2
#define MOTOR3_PWM  FTM2_CH2_PIN	// PTC3
#define MOTOR4_PWM  FTM2_CH3_PIN // PTC4
#define MOTOR_HZ    (20*1000)	//滑行模式下，频率应该是 30~100。
								//常规模式下，频率应该是 20k 左右
#define MOTOR_MAX   950
extern float battery;
extern int32 MOTOR_Duty_Left, MOTOR_Duty_Right;
extern volatile float  Distance;
extern int MOTOR_Speed_Left, MOTOR_Speed_Right;
extern int32 MOTOR_Left_Acc, MOTOR_Right_Acc;
extern uint8 Run_Flag;
extern uint8 Stop_Flag;
extern uint8 Run_Stop;
extern uint8 Auto_Run;
extern char Left_Crazy, Right_Crazy;
extern char Mode_Set;
extern int32  Speed_Limit_Max,Speed_Limit_Min;
void Get_Angle(void);
extern float angle,Gyro_Balance;

extern uint8 Fres;
void Speed_Calculate(void);
extern uint8 Run_OK;
typedef struct
{
	float X;			 //比例系数
	float Y;			 //积分系数
	float Z;		 	 //微分系数

}S_FLOAT_XYZ ; 

typedef struct
{
	int16 X;			 //比例系数
	int16 Y;			 //积分系数
	int16 Z;		 	 //微分系数

}S_INT16_XYZ ; 

typedef struct
{
	int32 X;			 //比例系数
	int32 Y;			 //积分系数
	int32 Z;		 	 //微分系数

}S_INT32_XYZ ; 

extern S_FLOAT_XYZ 
	GYRO_Real,		// 陀螺仪转化后的数据
	ACC_Real,		// 加速度计转化后的数据
	Attitude_Angle,	// 当前角度 
	Last_Angle,		// 上次角度
	Target_Angle;	// 目标角度
	

extern S_INT16_XYZ
	GYRO,			// 陀螺仪原始数据
	GYRO_Offset,	// 陀螺仪温飘
	GYRO_Last,		// 陀螺仪上次数据
	ACC, 			// 加速度计数据
	ACC_Offset,		// 加速度计温飘
	ACC_Last;		// 加速度计上次数据
extern S_INT32_XYZ
	Tar_Ang_Vel,	// 目标角速度
	Tar_Ang_Vel_Last;	// 上次目标角速度

extern int32 
	Speed_Now,		// 当前实际速度
	Speed_Min,		// 左右最小速度
	Speed_Set, 		// 目标设定速度
	Theory_Duty,
	Direct_Last,
	Speed_Diff,		// 当前实际速度
	Vel_Set,		// 目标转向角速度
	Direct_Parameter,
	Radius;

/*********** 函数声明 ************/
void Speed_Measure(void);	//电机速度测量
void Start_Control(void);	//起跑线检测与停车控制
void Speed_Control(void);
void MOTOR_Control(int32 LDuty, int32 RDuty);	// 电机控制
int32 range_protect(int32 duty, int32 min, int32 max); //限幅保护
void Diff_speedcontrol(void);
void WCZ_Data_Calc(u8 dT_ms);
void Speed_cal(void);
float Slope_Calculate(uint8 begin,uint8 end,float *p);
void BAT_CHECK(void);
void Balance_Init(void);
#endif
