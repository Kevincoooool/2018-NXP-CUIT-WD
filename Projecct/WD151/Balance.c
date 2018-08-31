#include "Balance.h"
#include "MPU6050.h"
#include "ANO_Data_Transfer.h"
#include "System.h"
#include "MyBalance.h"
#include "ADRC.h"
#include "PID.h"
#include "Control.h"
#include "Search.h"
#include "EM.h"
#include "show.h"

/********************************************************/
/********************* 串级平衡控制 *********************/
// 频率控制在定时器中设置
void Balance_Control(void)
{
//	MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
//	Data_steepest();
	Speed_Calculate();
	Radius = -PlacePID_Control(&Turn_PID, Turn[Fres], Middle_Err, 0);	
//	IMU_update(0.005f,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data); //姿态解算
		/* 角速度环作为最内环控制直立 */
	//ADRC_Control(&ADRC_GYRO_Controller,Tar_Ang_Vel.Y,GYRO_Real.Y*10);
	//Theory_Duty = ADRC_GYRO_Controller.u;
	Theory_Duty += -PID_Increase(&Ang_gyro_PID, Ang_gyro, (int32)(GYRO_Real.Y*10), 0);	// 计算直立PWM
	Theory_Duty = range_protect(Theory_Duty, -950, 950);
						/* 角速度环作为最内环控制转向 */									//Speed_Min
	
//	ADRC_Control(&ADRC_I_TURN_Controller,0,GYRO_Real.Z*10);
//	Direct_Parameter = -ADRC_I_TURN_Controller.u;
	Direct_Parameter = -PID_Realize(&Turn_gyro_PID, Turn_gyro, (int32)(GYRO_Real.Z*100), 0);	// 转向环左正右负
	Direct_Parameter = range_protect(Direct_Parameter, -1200, 1200);
	//Direct_Last = Direct_Last*0.2 + Direct_Parameter*0.8;	// 更新上次角速度环结果
	MOTOR_Duty_Left  = Theory_Duty - Direct_Last;	// 左右电机根据转向系数调整差速
	MOTOR_Duty_Right = Theory_Duty + Direct_Last;	
	
	if (Run_Flag)
	{
		MOTOR_Control(MOTOR_Duty_Right, MOTOR_Duty_Left);	// 控制左右电机
	}
	else
	{
		if (Stop_Flag)
		{
			if (Speed_Now > 80)
			{
				MOTOR_Control(550, 550);
			}
			else
			{
				MOTOR_Control(0, 0);
			}
		}
		else
		{
			MOTOR_Control(0, 0);
		}
	}	
	if (Angle_Flag)		// 直立角度环	10ms
	{
		Angle_Flag = 0;
		Speed_Measure();// 获取当前速度
				
		Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle, imu_data.pit*100, Target_Angle.Y);	/* 角度环加到角速度环上串级控制 */
		Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y, -1500, 1500);	// 注意正负号
	}
	if (Speed_Flag)		// 速度环	50ms
	{
		Speed_Flag = 0;
		Target_Angle.Y = PID_Realize(&Speed_PID, Speed, Speed_Now,0);/* 速度环加到角度环上串级控制 */
		Target_Angle.Y -= 2000;
		Target_Angle.Y = range_protect(Target_Angle.Y, -2700, 1500);	// -44 22

	}
}

/* 初始化用到的一些变量 */


